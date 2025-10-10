import dataclasses
from copy import deepcopy

import numpy as np
import rustworkx as rx
from box import Box
from probabilistic_model.distributions import (
    DiracDeltaDistribution,
    GaussianDistribution,
)
from probabilistic_model.distributions.helper import make_dirac
from probabilistic_model.probabilistic_circuit.rx.helper import (
    uniform_measure_of_event,
    leaf,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    SumUnit,
    ProbabilisticCircuit,
    ProductUnit,
)
from random_events.interval import closed
from random_events.polytope import Polytope, NoOptimalSolutionError
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Continuous
from scipy.spatial import ConvexHull
from semantic_world.datastructures.variables import SpatialVariables
from semantic_world.robots import AbstractRobot
from semantic_world.spatial_types import Point3
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import World
from semantic_world.world_description.connections import FixedConnection
from semantic_world.world_description.geometry import BoundingBox
from semantic_world.world_description.graph_of_convex_sets import GraphOfConvexSets
from semantic_world.world_description.shape_collection import BoundingBoxCollection
from semantic_world.world_description.world_entity import Body
from sortedcontainers import SortedSet
from typing_extensions import List, Union, Iterable, Optional, Iterator, Tuple

from ..config.action_conf import ActionConfig
from ..costmaps import (
    OccupancyCostmap,
    VisibilityCostmap,
    SemanticCostmap,
    GaussianCostmap,
    Costmap,
)
# from ..datastructures.dataclasses import BoundingBox, AxisAlignedBoundingBox, BoundingBoxCollection
from ..datastructures.enums import (
    Arms,
    Grasp,
    ApproachDirection,
    VerticalAlignment,
)
from ..datastructures.grasp import GraspDescription
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import PoseStamped, GraspPose, Vector3
from ..designator import LocationDesignatorDescription
from ..failures import RobotInCollision
from ..pose_generator_and_validator import (
    PoseGenerator,
    visibility_validator,
    pose_sequence_reachability_validator,
    OrientationGenerator,
    collision_check,
)
from ..robot_description import ViewManager
from ..ros import logerr
from ..utils import translate_pose_along_local_axis
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


def _create_target_sequence(
        grasp_description: GraspDescription,
        target: Union[PoseStamped, Body],
        robot: AbstractRobot,
        object_in_hand: Body,
        reachable_arm: Arms,
        world: World,
        rotation_agnostic: bool = False,
) -> List[PoseStamped]:
    """
    Creates the sequence of poses that need to be reachable for the robot to grasp the target.
    For pickup this would be retract pose, target pose and lift pose.
    For place this would be lift pose, target pose and retract pose.


    :param grasp_description: Grasp description to be used for grasping
    :param target: The target of reachability, either a pose or an object
    :param robot: The robot that should be checked for reachability
    :param object_in_hand: An object that is held if any
    :param reachable_arm: The arm which should be checked for reachability
    :return: A list of poses that need to be reachable in this order
    """
    end_effector = ViewManager.get_end_effector_view(reachable_arm, robot)

    grasp_quaternion = grasp_description.calculate_grasp_orientation(
        end_effector.front_facing_orientation.to_np()
    )
    approach_axis = end_effector.front_facing_axis

    target_pose = (
        deepcopy(target)
        if isinstance(target, PoseStamped)
        else grasp_description.get_grasp_pose(end_effector, target)
    )
    # target.get_grasp_pose(end_effector, grasp_description)

    if object_in_hand:
        if rotation_agnostic:
            robot_rotation = PoseStamped.from_spatial_type(
                robot.root.global_pose
            ).orientation
            target_pose.orientation = robot_rotation
            approach_direction = GraspDescription(
                grasp_description.approach_direction,
                VerticalAlignment.NoAlignment,
                False,
            )
            side_grasp = np.array(
                grasp_description.calculate_grasp_orientation(
                    end_effector.front_facing_orientation.to_np()
                )
            )
            side_grasp *= np.array([-1, -1, -1, 1])
            target_pose.rotate_by_quaternion(side_grasp.tolist())

        target_pose = PoseStamped.from_spatial_type(world.transform(
            object_in_hand.global_pose,
            target_pose.frame_id,
        ))

        # TODO: Find a good way to handle the approach offsets
        # approach_offset_cm = object_in_hand.get_approach_offset()
        approach_offset_cm = 0.1
    else:

        target_pose.rotate_by_quaternion(grasp_quaternion)
        approach_offset_cm = (
            0.1  # if isinstance(target, PoseStamped) else target.get_approach_offset()
        )

    lift_pose = deepcopy(target_pose)
    lift_pose.position.z += 0.1

    retract_pose = translate_pose_along_local_axis(
        target_pose, approach_axis.to_np()[:3], -approach_offset_cm
    )

    target_sequence = (
        [lift_pose, target_pose, retract_pose]
        if object_in_hand
        else [retract_pose, target_pose, lift_pose]
    )

    return target_sequence


@dataclasses.dataclass
class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    def __init__(
            self,
            target: Union[PoseStamped, Body],
            reachable_for: AbstractRobot = None,
            visible_for: AbstractRobot = None,
            reachable_arm: Optional[Union[Iterable[Arms], Arms]] = None,
            ignore_collision_with: Optional[Union[Iterable[Body], Body]] = None,
            grasp_descriptions: Optional[
                Union[Iterable[GraspDescription], GraspDescription]
            ] = None,
            # object_in_hand: Optional[Union[Iterable[Body], Body]] = None,
            rotation_agnostic: bool = False,
    ):
        """
        Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
        visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

        :param target: Location for which visibility or reachability should be calculated
        :para world: The world in which the location should be calculated
        :param reachable_for: Object for which the reachability should be calculated, usually a robot
        :param visible_for: Object for which the visibility should be calculated, usually a robot
        :param reachable_arm: An optional arm with which the target should be reached
        :param ignore_collision_with: List of objects that should be ignored for collision checking.
        :param grasp_descriptions: List of grasps that should be tried to reach the target pose
        :param object_in_hand: Object that is currently in the hand of the robot
        :param rotation_agnostic: If True, the target pose is adjusted so that it is pointing to the robot
        """
        super().__init__()
        PartialDesignator.__init__(
            self,
            CostmapLocation,
            target=target,
            reachable_for=reachable_for,
            visible_for=visible_for,
            reachable_arm=(
                reachable_arm if reachable_arm is not None else [Arms.LEFT, Arms.RIGHT]
            ),
            ignore_collision_with=(
                ignore_collision_with if ignore_collision_with is not None else [[]]
            ),
            grasp_descriptions=(
                grasp_descriptions if grasp_descriptions is not None else [None]
            ),
            rotation_agnostic=rotation_agnostic,
        )
        self.target: Union[PoseStamped, Body] = target
        self.reachable_for: Body = reachable_for
        self.visible_for: Body = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.ignore_collision_with = (
            ignore_collision_with if ignore_collision_with is not None else [[]]
        )
        self.grasps: List[Optional[Grasp]] = (
            grasp_descriptions if grasp_descriptions is not None else [None]
        )

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def setup_costmaps(
            self, target: PoseStamped, visible_for, reachable_for
    ) -> Costmap:
        """
        Sets up the costmaps for the given target and robot. The costmaps are merged and stored in the final_map


        """
        ground_pose = deepcopy(target)
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.32, self.world, 200, 0.02, ground_pose)
        final_map = occupancy

        if visible_for:
            camera = list(self.robot_view.neck.sensors)[0]
            visible = VisibilityCostmap(
                camera.minimal_height,
                camera.maximal_height,
                self.world,
                200,
                0.02,
                target,
            )
            final_map += visible

        if reachable_for:
            gaussian = GaussianCostmap(200, 15, self.world, 0.02, ground_pose)
            final_map += gaussian

        return final_map

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
            test_world = deepcopy(self.world)
            test_world.name = "Test World"

            params_box = Box(params)
            # Target is either a pose or an object since the object is later needed for the visibility validator
            target = (
                deepcopy(params_box.target)
                if isinstance(params_box.target, PoseStamped)
                else PoseStamped.from_spatial_type(params_box.target.global_pose)
            )

            if params_box.visible_for or params_box.reachable_for:
                robot_object = (
                    params_box.visible_for
                    if params_box.visible_for
                    else params_box.reachable_for
                )
            else:
                robot_object = ViewManager.find_active_robots_for_world(test_world)

            test_robot = ViewManager.get_view_in_other_world(robot_object, test_world)

            objects_in_hand = list(
                set(
                    test_world.get_kinematic_structure_entities_of_branch(
                        test_robot.root
                    )
                )
                - set(test_robot.bodies)
            )
            object_in_hand = objects_in_hand[0] if objects_in_hand else None

            final_map = self.setup_costmaps(
                target, params_box.visible_for, params_box.reachable_for
            )

            for pose_candidate in PoseGenerator(final_map, number_of_samples=600):
                pose_candidate.position.z = 0
                test_robot.root.parent_connection.origin = (
                    pose_candidate.to_spatial_type()
                )

                collisions = collision_check(
                    test_robot,
                    allowed_collision=params_box.ignore_collision_with,
                    world=test_world,
                )

                if collisions:
                    continue

                if not (params_box.reachable_for or params_box.visible_for):
                    yield pose_candidate
                    continue

                if params_box.visible_for and not visibility_validator(
                        test_robot, target, test_world
                ):
                    continue

                if not params_box.reachable_for:
                    yield pose_candidate
                    continue

                grasp_descriptions = (
                    [params_box.grasp_descriptions]
                    if params_box.grasp_descriptions
                    else GraspDescription.calculate_grasp_descriptions(
                        test_robot, target
                    )
                )

                for grasp_desc in grasp_descriptions:

                    target_sequence = _create_target_sequence(
                        grasp_desc,
                        target,
                        test_robot,
                        object_in_hand,
                        params_box.reachable_arm,
                        params_box.rotation_agnostic,
                    )
                    ee = ViewManager.get_arm_view(params_box.reachable_arm, test_robot)
                    is_reachable = pose_sequence_reachability_validator(
                        test_world.root,
                        ee.manipulator.tool_frame,
                        target_sequence,
                        allowed_collision=params_box.ignore_collision_with,
                        world=test_world,
                    )
                    if is_reachable:
                        yield GraspPose(
                            pose_candidate.pose,
                            pose_candidate.header,
                            arm=params_box.reachable_arm,
                            grasp_description=grasp_desc,
                        )


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """

    def __init__(
            self,
            handle: Union[Body, Iterable[Body]],
            robot_desig: Union[AbstractRobot, Iterable[AbstractRobot]],
            arm: Union[List[Arms], Arms] = None,
            prepose_distance: float = ActionConfig.grasping_prepose_distance,
    ):
        """
        Describes a position from where a drawer can be opened. For now this position should be calculated before the
        drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

        :param handle: ObjectPart designator for handle of the drawer
        :param robot_desig: Object designator for the robot which should open the drawer
        """
        super().__init__()
        PartialDesignator.__init__(
            self,
            AccessingLocation,
            handle=handle,
            robot_desig=robot_desig,
            arm=arm if arm is not None else [Arms.LEFT, Arms.RIGHT],
            prepose_distance=prepose_distance,
        )
        self.handle: Body = handle
        self.robot: AbstractRobot = robot_desig
        self.prepose_distance = prepose_distance
        self.arm = arm if arm is not None else [Arms.LEFT, Arms.RIGHT]

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators for this location designator, just returns the first element from the iteration

        :return: A location designator for a pose from which the drawer can be opened
        """
        return next(iter(self))

    @staticmethod
    def adjust_map_for_drawer_opening(
            cost_map: Costmap,
            init_pose: PoseStamped,
            goal_pose: PoseStamped,
            width: float = 0.2,
    ):
        """
        Adjust the cost map for opening a drawer. This is done by removing all locations between the initial and final
        pose of the drawer/container.

        :param cost_map: Costmap that should be adjusted.
        :param init_pose: Pose of the drawer/container when it is fully closed.
        :param goal_pose: Pose of the drawer/container when it is fully opened.
        :param width: Width of the drawer/container.
        """
        motion_vector = [
            goal_pose.position.x - init_pose.position.x,
            goal_pose.position.y - init_pose.position.y,
            goal_pose.position.z - init_pose.position.z,
        ]
        # remove locations between the initial and final pose
        motion_vector_length = np.linalg.norm(motion_vector)
        unit_motion_vector = np.array(motion_vector) / motion_vector_length
        orthogonal_vector = np.array([unit_motion_vector[1], -unit_motion_vector[0], 0])
        orthogonal_vector /= np.linalg.norm(orthogonal_vector)
        orthogonal_size = width
        map_origin_idx = cost_map.map.shape[0] // 2, cost_map.map.shape[1] // 2
        for i in range(int(motion_vector_length / cost_map.resolution)):
            for j in range(int(orthogonal_size / cost_map.resolution)):
                idx = (
                    int(
                        map_origin_idx[0]
                        + i * unit_motion_vector[0]
                        + j * orthogonal_vector[0]
                    ),
                    int(
                        map_origin_idx[1]
                        + i * unit_motion_vector[1]
                        + j * orthogonal_vector[1]
                    ),
                )
                cost_map.map[idx] = 0
                idx = (
                    int(
                        map_origin_idx[0]
                        + i * unit_motion_vector[0]
                        - j * orthogonal_vector[0]
                    ),
                    int(
                        map_origin_idx[1]
                        + i * unit_motion_vector[1]
                        - j * orthogonal_vector[1]
                    ),
                )
                cost_map.map[idx] = 0

    def setup_costmaps(self, handle: Body) -> Costmap:
        """
        Sets up the costmaps for the given handle and robot. The costmaps are merged and stored in the final_map.
        """
        ground_pose = PoseStamped.from_spatial_type(handle.global_pose)
        ground_pose.position.z = 0
        occupancy = OccupancyCostmap(
            distance_to_obstacle=0.25,
            size=200,
            resolution=0.02,
            origin=ground_pose,
            world=handle._world,
        )
        final_map = occupancy

        gaussian = GaussianCostmap(200, 15, handle._world, 0.02, ground_pose)
        final_map += gaussian

        return final_map

    def create_target_sequence(
            self, params_box: Box, final_map: Costmap
    ) -> List[PoseStamped]:
        """
        Creates the sequence of target poses

        :param params_box:
        :param final_map:
        :return:
        """
        handle = params_box.handle
        # compute the chain of connections only works top down,
        handle_to_root_connections = list(
            reversed(
                handle._world.compute_chain_of_connections(handle._world.root, handle)
            )
        )
        # Search for the first connection that is not a FixedConnection,
        container_connection = list(
            filter(
                lambda c: not isinstance(c, FixedConnection), handle_to_root_connections
            )
        )[0]

        lower_limit = container_connection.dof.lower_limits.position
        upper_limit = container_connection.dof.upper_limits.position

        init_pose = link_pose_for_joint_config(
            params_box.handle, {container_connection.dof.name.name: lower_limit}
        )

        # Calculate the pose the handle would be in if the drawer was to be fully opened
        goal_pose = link_pose_for_joint_config(
            params_box.handle, {container_connection.dof.name.name: upper_limit}
        )

        # Handle position for calculating rotation of the final pose
        half_pose = link_pose_for_joint_config(
            params_box.handle, {container_connection.dof.name.name: upper_limit / 1.5}
        )

        # joint_type = params_box.handle.parent_entity.joints[container_joint].type

        # if joint_type == JointType.PRISMATIC:
        #     self.adjust_map_for_drawer_opening(final_map, init_pose, goal_pose)

        target_sequence = [init_pose, half_pose, goal_pose]
        return target_sequence

    def __iter__(self) -> Iterator[PoseStamped]:
        """
        Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
        handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
        the handle can be grasped if the drawer is open.

        :yield: A location designator containing the pose and the arms that can be used.
        """
        test_world = deepcopy(self.world)
        test_robot = self.robot_view.from_world(test_world)
        for params in self.generate_permutations():
            params_box = Box(params)

            final_map = self.setup_costmaps(params_box.handle)

            target_sequence = self.create_target_sequence(params_box, final_map)
            half_pose = target_sequence[1]

            orientation_generator = lambda p, o: PoseGenerator.generate_orientation(
                p, half_pose
            )
            for pose_candidate in PoseGenerator(
                    final_map,
                    number_of_samples=600,
                    orientation_generator=orientation_generator,
            ):
                pose_candidate.position.z = 0
                test_robot.root.parent_connection.origin = (
                    pose_candidate.to_spatial_type()
                )
                try:
                    collision_check(test_robot, [], test_world)
                except RobotInCollision:
                    continue

                for arm_chain in test_robot.manipulator_chains:
                    grasp = GraspDescription(
                        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                    ).calculate_grasp_orientation(
                        arm_chain.manipulator.front_facing_orientation.to_np()
                    )
                    current_target_sequence = [
                        deepcopy(pose) for pose in target_sequence
                    ]
                    for pose in current_target_sequence:
                        pose.rotate_by_quaternion(grasp)

                    is_reachable = pose_sequence_reachability_validator(
                        test_world.root,
                        arm_chain.manipulator.tool_frame,
                        current_target_sequence,
                        world=test_world,
                    )
                    if is_reachable:
                        yield pose_candidate


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    def __init__(
            self,
            body,
            for_object: Body = None,
            edges_only: bool = False,
            horizontal_edges_only: bool = False,
            edge_size_in_meters: float = 0.06,
            height_offset: float = 0.0,
    ):
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
        PartialDesignator.__init__(
            self,
            SemanticCostmapLocation,
            body=body,
            for_object=for_object,
            edges_only=edges_only,
            horizontal_edges_only=horizontal_edges_only,
            edge_size_in_meters=edge_size_in_meters,
            height_offset=height_offset,
        )
        self.body: Body = body
        self.for_object: Optional[Body] = for_object
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

            self.sem_costmap = SemanticCostmap(params_box.body)
            if params_box.edges_only or params_box.horizontal_edges_only:
                self.sem_costmap = self.sem_costmap.get_edges_map(
                    params_box.edge_size_in_meters,
                    horizontal_only=params_box.horizontal_edges_only,
                )
            height_offset = params_box.height_offset
            if params_box.for_object:
                bb_points = (
                    params_box.for_object.collision.as_bounding_box_collection_in_frame(
                        params_box.for_object
                    )
                    .bounding_box()
                    .get_points()
                )
                np_points = [point.to_np()[:3] for point in bb_points]
                min_z = min(np_points, key=lambda p: p[2])[2]
                max_z = max(np_points, key=lambda p: p[2])[2]
                height_offset = (max_z - min_z) / 2
            for maybe_pose in PoseGenerator(self.sem_costmap):
                maybe_pose.position.z += height_offset
                yield maybe_pose


class ProbabilisticSemanticLocation(LocationDesignatorDescription):
    """
    Location designator which samples poses from a semantic costmap with a probability distribution.
    The construction of the probabilistic circuit can be quite time-consuming, but the majority of the computational load
    is done during the first iteration only. In this case the exact time is highly dependent on the number of
    links in the world, and the number of links we are sampling from.
    """

    surface_x = Continuous("surface_x")
    """
    Variable representing the x coordinate on a surface
    """

    surface_y = Continuous("surface_y")
    """
    Variable representing the y coordinate on a surface
    """

    def __init__(
            self,
            bodies: List[Body],
            for_object: Body = None,
            link_is_center_link: bool = False,
            number_of_samples: int = 1000,
            sort_sampels: bool = False,
            uniform_sampling: bool = False,
            highlight_used_surfaces: bool = False,
    ):
        """
        Creates a distribution over a link to sample poses which are on this link. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        :param for_object: Optional object that should be placed at the found location
        :param link_is_center_link: If True, the link is considered the center link of the object, otherwise it is
        considered the surface link. This is important as some urdf models only model the center link and not the surface link.
        :param number_of_samples: Number of samples to be generated from the surface
        :param sort_sampels: If True, the samples are sorted by their probability
        :param uniform_sampling: If True, the samples are uniformly distributed over the surface, otherwise they are
        sampled from a Gaussian distribution centered around the surface samples.
        :param highlight_used_surfaces: If True, the surfaces used for sampling temporarily have their color changed
        """
        super().__init__()
        PartialDesignator.__init__(
            self, ProbabilisticSemanticLocation, bodies=[bodies], for_object=for_object
        )
        self.sort_samples = sort_sampels
        self.uniform_sampling = uniform_sampling
        self.number_of_samples = number_of_samples
        self.bodies: List[Body] = bodies
        self.link_is_center_link: bool = link_is_center_link
        self.for_object: Optional[Body] = for_object
        self.highlight_used_surfaces: bool = highlight_used_surfaces

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def _create_link_circuit(
            self,
            surface_samples: List[Tuple[float, float]],
    ) -> ProbabilisticCircuit:
        """
        Creates a probabilistic circuit that samples navigation poses on a surface defined by the given surface samples.
        The circuit will sample poses that are close to the surface samples and have the given link id as true.
        The circuit will also sample the x and y coordinates of the poses from a Gaussian distribution centered around
        the surface samples with a scale of 1.0.

        :param surface_samples: A list of surface samples, each sample is a tuple of (x, y) coordinates.

        :return: A ProbabilisticCircuit that samples navigation poses on the surface defined by the surface samples.
        """
        link_circuit = ProbabilisticCircuit()
        link_circuit_root = SumUnit(probabilistic_circuit=link_circuit)
        scale = 1.0

        for surface_sample in surface_samples[:100]:
            p_point_root = ProductUnit(probabilistic_circuit=link_circuit)
            link_circuit_root.add_subcircuit(p_point_root, 1.0)

            # We are looking for a navigation sample, for which the surface sample and the link id are true (reachable)
            surface_x_p = make_dirac(self.surface_x, surface_sample[0])
            surface_y_p = make_dirac(self.surface_y, surface_sample[1])
            # target_link_id_p = make_dirac(link_id_symbol, link_id)

            # We create a Gaussian distribution around the surface sample, which is used to sample the
            # navigation poses.
            x_p = GaussianDistribution(SpatialVariables.x.value, surface_sample[0], scale)
            y_p = GaussianDistribution(SpatialVariables.y.value, surface_sample[1], scale)

            # Add all the variables to the circuit
            p_point_root.add_subcircuit(leaf(surface_x_p, link_circuit))
            p_point_root.add_subcircuit(leaf(surface_y_p, link_circuit))
            p_point_root.add_subcircuit(leaf(x_p, link_circuit))
            p_point_root.add_subcircuit(leaf(y_p, link_circuit))

        link_circuit.normalize()

        return link_circuit

    @staticmethod
    def _create_surface_event(
            body: Body, search_space: BoundingBoxCollection, wall_bloat: float
    ) -> Optional[Event]:
        """
        Creates an event that describes the surface of the link we want to sample from.
        The surface event is constructed from the bounding box of the link, and the walls and doors are cut out to
        ensure that the target poses are not too close to walls or doors.

        :param body: The link for which the surface event should be created.
        :param search_space: The search space in which the surface event should be created.
        :param wall_bloat: The amount by which the walls should be bloated to ensure the target poses are not too close
        to walls or doors.

        :return: An Event that describes the surface of the link, or None if the surface event is empty.
        """
        # Construct the surface event, which is the event that describes the surface of the link we want to
        # sample from.
        # We cut out the walls and doors from the surface event, to ensure the target poses are not too close
        # to walls or doors, since they are harder to reach.
        global_root = body._world.root
        surface_event = body.collision.as_bounding_box_collection_in_frame(global_root).event
        obstacle_boxes = list(
            filter(
                None,
                [
                    (
                        b.collision.as_bounding_box_collection_in_frame(
                            global_root
                        ).bounding_box()
                        if any(obstacle in b.name.name.lower() for obstacle in
                               {"wall", "door"}) and b.collision.shapes != []
                        else None
                    )
                    for b in body._world.bodies
                ],
            )
        )
        obstacle_collection = BoundingBoxCollection(shapes=obstacle_boxes, reference_frame=global_root)
        walls = GraphOfConvexSets.obstacles_from_bounding_boxes(
            obstacle_collection,
            search_space_event=search_space.event,
        )

        if walls is not None:
            surface_event = ~walls & surface_event

        if surface_event.is_empty():
            return None

        surface_event = surface_event.marginal(
            SpatialVariables.xy
        )

        return surface_event

    def _create_navigation_space_event(
            self, free_space: GraphOfConvexSets, body: Body
    ) -> Optional[Event]:
        """
        Creates an event that describes the navigation space for the link we want to sample from.
        The navigation space is the free space around the link, which is used to sample navigation poses.
        The navigation space is constructed from the free space graph, which is a subgraph of the free space that is
        reachable from the target node, which is the node in the free space graph that is above the link.

        :param free_space: The free space graph that is used to sample navigation poses.
        :param body: The link for which the navigation space event should be created.

        :return: An Event that describes the navigation space for the link, or None if the navigation space event is empty.
        """
        # Event that describes the robot standing on the ground
        stand_on_ground = SimpleEvent(
            {SpatialVariables.z.value: (0, 0.05)}
        ).as_composite_set()
        stand_on_ground.fill_missing_variables(
            SpatialVariables.xy,
        )

        # target_node is the node in the free space graph that is above the link
        # We add a small offset to the z-coordinate to ensure that we are above the link even with a small bloat
        # The target node is used classify which parts of the free space graph are relevant for the link
        # (the navigation_space_graph), and which parts we can discard (for example because there is a wall
        # cutting through the search space)
        z_offset = (
            (body.collision.as_bounding_box_collection_in_frame(body._world.root).bounding_box().height / 2) + 0.1
            if self.link_is_center_link
            else 0.1
        )
        target_point = body.global_pose.to_position().to_np()[:3]
        target_point[2] += z_offset
        target_node = free_space.node_of_point(Point3(*target_point, reference_frame=body._world.root))
        if target_node is None:
            return None

        all_paths = rx.dijkstra_shortest_paths(free_space.graph, free_space.box_to_index_map[target_node])
        navigation_space_graph = free_space.create_subgraph([path for path in all_paths])
        navigation_space_event = Event(
            *[node.simple_event for node in navigation_space_graph.graph.nodes()]
        )
        navigation_space_event.fill_missing_variables(
            SortedSet([self.surface_x, self.surface_y])
        )

        # We want to sample from the navigation space, but only if the robot is standing on the ground
        navigation_space_event &= stand_on_ground

        if navigation_space_event.is_empty():
            return navigation_space_event

        navigation_space_event = navigation_space_event.marginal(
            SpatialVariables.xy
        )
        return navigation_space_event

    def _create_distribution_for_link(
            self, body: Body
    ) -> Tuple[Optional[ProbabilisticCircuit], float]:
        """
        Creates a distribution for the given link, which is a probabilistic circuit that samples navigation poses on the
        surface of the link. The distribution is conditioned on the navigation space event, which is the free space
        around the link, and the surface event, which is the surface of the link we want to sample from.

        :param body: The link for which the distribution should be created.

        :return: A tuple containing the conditioned link circuit and the probability of the condition.
        """

        # Create a search space around the link, which is the axis aligned bounding box of the link, bloated by
        # 1 meter in all directions
        # link_searchspace = link.get_axis_aligned_bounding_box().bloat(1, 1, 1).as_collection()
        link_searchspace = body.collision.as_bounding_box_collection_in_frame(body._world.root).bloat(1.5, 1.5, 1.5)

        # Calculate the minimal maximum bloat we can apply to the walls of the link
        # since our target node is above the link, bloating the walls too much causes the target node to be
        # unreachable
        link_width = (
            body.collision.as_bounding_box_collection_in_frame(body._world.root)
            .bounding_box()
            .width
        )
        link_depth = (
            body.collision.as_bounding_box_collection_in_frame(body._world.root)
            .bounding_box()
            .depth
        )
        link_min_dim = link_width if link_width < link_depth else link_depth
        wall_bloat = 0.3 if link_min_dim > 0.8 else link_min_dim / 2.5

        # Create the free space graph for the link, which is the free space around the link
        # The obstacles are bloated by 0.02 meters in x and y, and 0.01 meters in z, to close any unintended gaps
        # in the urdf.
        # The walls are bloated to ensure that the navigation poses are not too close to the walls
        free_space = GraphOfConvexSets.free_space_from_world(
            body._world,
            link_searchspace,
            bloat_obstacles=0.02,
        )

        surface_event = self._create_surface_event(body, link_searchspace, wall_bloat)
        if surface_event is None:
            return None, 0.0

        navigation_space_event = self._create_navigation_space_event(free_space, body)
        if navigation_space_event is None:
            return None, 0.0

        # We don't want to navigate into the surface, so we cut out the surface event from the navigation_space_event
        navigation_space_event = ~surface_event & navigation_space_event

        surface_measure = uniform_measure_of_event(surface_event)

        surface_samples = surface_measure.sample(100)
        surface_measure.log_likelihood(surface_samples)

        link_circuit = self._create_link_circuit(
            surface_samples[:100]
        )

        surface_event.fill_missing_variables(link_circuit.variables)

        # Condition the circuit on the navigation space event
        return link_circuit.truncated(navigation_space_event)

    @staticmethod
    def _calculate_surface_z_coord(
            test_robot: AbstractRobot, surface_coords: Tuple[float, float]
    ) -> Optional[float]:
        """
        Calculates the z-coordinate of the surface at the given surface coordinates on the link.

        :param test_robot: The robot that is used to test the surface coordinates.
        :param surface_coords: The coordinates on the surface where the z-coordinate should be calculated.

        :return: The z-coordinate of the surface at the given surface coordinates, or None if the link is not hit.
        """
        # Use a raytest to check if the sampled point is on the target link. This is necessary because the
        # bounding boxes are not perfect, so the sampled point may be slightly off the link.
        # Furthermore, we can use the ray test to get the correct height of the link at the sampled point.
        surface_x_coord, surface_y_coord = surface_coords
        ray_start = [surface_x_coord, surface_y_coord, 2]
        ray_end = [surface_x_coord, surface_y_coord, -2.0]
        result = test_robot._world.ray_tracer.ray_test(np.array(ray_start), np.array(ray_end))
        hit_pos = result[0][0][2]

        return hit_pos

    def __iter__(self) -> Iterator[PoseStamped]:
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: A PoseStamped with the found valid position of the Costmap.
        """
        test_world = deepcopy(self.world)
        test_world.name = "Test World"
        test_robot = self.robot_view.from_world(test_world)

        for params in self.generate_permutations():

            params_box = Box(params)

            if self.highlight_used_surfaces:
                # Highlight the links we are sampling from
                old_colors = [
                    params_box.part_of.get_link_color(link).get_rgb()
                    for link in params_box.link_names
                ]
                HIGHLIGHT_COLOUR = [1.0, 0.0, 1.0]
                for name in params_box.link_names:
                    params_box.part_of.set_link_color(name, HIGHLIGHT_COLOUR)

            world_pc = ProbabilisticCircuit()
            world_distribution = SumUnit(probabilistic_circuit=world_pc)

            for body in params_box.bodies:

                conditioned_link_circuit, p_condition = (
                    self._create_distribution_for_link(body)
                )

                if conditioned_link_circuit is None:
                    continue

                if self.uniform_sampling:
                    log_weight = 0.0

                else:
                    log_weight = np.log(p_condition)

                temp_root = conditioned_link_circuit.root
                remap = world_pc.add_from_subgraph(conditioned_link_circuit.graph)
                world_distribution.add_subcircuit(remap[temp_root.index], log_weight)

            world_distribution.probabilistic_circuit.normalize()

            samples = world_distribution.probabilistic_circuit.sample(
                self.number_of_samples
            )

            if self.sort_samples:
                samples = samples[
                    np.argsort(
                        world_distribution.probabilistic_circuit.log_likelihood(samples)
                    )[::-1]
                ]
            else:
                np.random.shuffle(samples)

            if self.highlight_used_surfaces:
                for name, old_color in zip(params_box.link_names, old_colors):
                    params_box.part_of.set_link_color(name, old_color)

            # TODO: Use Prospection World here in the future, but currently there is annoying context management, causing
            #       subsequent actions to be executed in the Prospection World, causing problems with detecting etc
            for sample in samples:
                sample_dict = {
                    var.name: val
                    for var, val in zip(
                        world_distribution.probabilistic_circuit.variables, sample
                    )
                }
                surface_x_coord = sample_dict[self.surface_x.name]
                surface_y_coord = sample_dict[self.surface_y.name]
                nav_x = sample_dict[SpatialVariables.x.name]
                nav_y = sample_dict[SpatialVariables.y.name]

                surface_z_coord = self._calculate_surface_z_coord(
                    test_robot, (surface_x_coord, surface_y_coord)
                )
                if surface_z_coord is None:
                    continue

                target_quat = OrientationGenerator.generate_random_orientation()
                target_pose = PoseStamped.from_list(
                    self.world.root,
                    [surface_x_coord, surface_y_coord, surface_z_coord],
                    target_quat,
                )

                nav_quat = OrientationGenerator.generate_origin_orientation(
                    [nav_x, nav_y], target_pose
                )
                nav_pose = PoseStamped.from_list(self.world.root, [nav_x, nav_y, 0], nav_quat)

                # Reject samples in which the robot is in collision with the environment despite the bloated obstacles,
                # for example with the arms
                test_robot.root.parent_connection.origin = nav_pose.to_spatial_type()
                try:
                    collision_check(test_robot, [], test_world)
                except RobotInCollision:
                    continue

                # Adjust the target pose if an object is given, so that the bottom of the object is on the link
                if params_box.for_object:
                    bounding_box: BoundingBox = params_box.for_object.collision.as_bounding_box_collection_in_frame(
                        self.world.root).bounding_box()
                    final_height_offset = (bounding_box.max_z - bounding_box.min_z) / 2
                    target_pose.position.z += final_height_offset

                yield target_pose


@dataclasses.dataclass
class ProbabilisticCostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains.
    The construction of the probabilistic circuit can be quite time-consuming, but the majority of the computational load
    is done during the first iteration only. In this case the exact time is highly dependent on the number of
    links in the world.
    """

    target_x = Continuous("target_x")
    """
    Variable representing the x coordinate of the target pose
    """

    target_y = Continuous("target_y")
    """
    Variable representing the y coordinate of the target pose
    """

    def __init__(
            self,
            target: Union[PoseStamped, Body],
            reachable_for: Optional[Union[Iterable[Body], Body]] = None,
            visible_for: Optional[Union[Iterable[Body], Body]] = None,
            reachable_arm: Optional[Union[Iterable[Arms], Arms]] = None,
            ignore_collision_with: Optional[Union[Iterable[Body], Body]] = None,
            grasp_descriptions: Optional[
                Union[Iterable[GraspDescription], GraspDescription]
            ] = None,
            object_in_hand: Optional[Union[Iterable[Body], Body]] = None,
            rotation_agnostic: bool = False,
            number_of_samples: int = 300,
            costmap_resolution: float = 0.1,
    ):
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
        :param number_of_samples: Number of samples to be generated from the costmap
        :param costmap_resolution: The edge length of the costmap cells in meters, defaults to 0.1 meters.
        """
        super().__init__()
        PartialDesignator.__init__(
            self,
            ProbabilisticCostmapLocation,
            target=target,
            reachable_for=reachable_for,
            visible_for=visible_for,
            reachable_arm=(
                reachable_arm if reachable_arm is not None else [Arms.LEFT, Arms.RIGHT]
            ),
            ignore_collision_with=(
                ignore_collision_with if ignore_collision_with is not None else [[]]
            ),
            grasp_descriptions=(
                grasp_descriptions if grasp_descriptions is not None else [None]
            ),
            object_in_hand=object_in_hand,
            rotation_agnostic=rotation_agnostic,
            number_of_samples=number_of_samples,
            costmap_resolution=costmap_resolution,
        )
        self.number_of_samples = number_of_samples
        # The resolution is divided by 2, since each sampled point is a center of a cell in the costmap
        self.costmap_resolution = costmap_resolution / 2
        self.target: Union[PoseStamped, Body] = target
        self.reachable_for: Body = reachable_for
        self.visible_for: Body = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.ignore_collision_with = (
            ignore_collision_with if ignore_collision_with is not None else [[]]
        )
        self.grasps: List[Optional[Grasp]] = (
            grasp_descriptions if grasp_descriptions is not None else [None]
        )

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    @staticmethod
    def _calculate_room_event(
            world: World, free_space_graph: GraphOfConvexSets, target_position: Vector3
    ) -> Event:
        """
        Calculates an event for the free space inside the room around the target position is located in, in 2d.

        :param world: The current world
        :param free_space_graph: The free space graph that is used as basis for the room calculation
        :param target_position: The position of the target in the world

        :return: An Event that describes the room around the target position in 2d
        """
        rays_end = np.array([
            [node.origin.to_np()[0, 3], node.origin.to_np()[1, 3], target_position.z + 0.2]
            for node in free_space_graph.graph.nodes()
        ])
        rays_start = np.array([
                                  [target_position.x, target_position.y, target_position.z + 0.2]
                              ] * len(rays_end))

        robot = world.get_views_by_type(AbstractRobot)[0]
        robot_pose = robot.root.global_pose
        robot.root.parent_connection.origin = TransformationMatrix.from_xyz_quaternion(100, 100, 0)

        test = world.ray_tracer.ray_test(rays_start, rays_end)
        robot.root.parent_connection.origin = robot_pose

        hit_positions = test[0]

        successful_hits = [
            [hx, hy] if (hx, hy) != (0.0, 0.0) else [rx, ry]
            for (hx, hy, _), (rx, ry, _) in zip(hit_positions, rays_end)
        ]
        hull = ConvexHull(successful_hits)
        A = hull.equations[:, :-1]
        b = -hull.equations[:, -1]

        # reason for this is the error
        # "E0000 00:00:1750253132.517928 1146896 linear_solver.cc:2026] No solution exists. MPSolverInterface::result_status_ = MPSOLVER_ABNORMAL"
        # this is not a great way to handle this i think, so we should find out in which situations this occurs @TODO
        poly = Polytope(A, b)
        try:
            room_event = poly.inner_box_approximation(minimum_volume=0.1)
        except NoOptimalSolutionError as e:
            logerr(f"No optimal solution found for Polytope: {poly}")
            room_event = poly.to_simple_event().as_composite_set()
        room_event = room_event.update_variables(
            {
                Continuous("x_0"): SpatialVariables.x.value,
                Continuous("x_1"): SpatialVariables.y.value,
            }
        )
        room_event.fill_missing_variables(SortedSet([SpatialVariables.z.value]))

        return room_event

    def _create_free_space_conditions(
            self, world: World, target_position: Vector3, search_distance: float = 1.5
    ) -> Tuple[Event, Event, Event]:
        """
        Creates the conditions for the free space around the target position.
        1. reachable_space_condition: The condition that describes the reachable space around the target position in 3d
        2. navigation_space_condition: The condition that describes the navigation space around the target position in 2d
        3. room_condition: The condition that describes the room around the target position in 2d

        :param world: The current world
        :param target_position: The position of the target in the world
        :param search_distance: The distance around the target position to search for free space, defaults to 1.5 meters

        :return: A tuple containing the reachable space condition, navigation space condition and room condition
        """

        link_searchspace = BoundingBoxCollection(
            [BoundingBox(
                target_position.x - search_distance,
                target_position.y - search_distance,
                0,
                target_position.x + search_distance,
                target_position.y + search_distance,
                target_position.z + 0.35, origin=self.world.root.global_pose)],
            reference_frame=self.world.root
        )

        free_space_nav = GraphOfConvexSets.navigation_map_from_world(
            world, search_space=link_searchspace, bloat_obstacles=0.3
        )

        free_space = GraphOfConvexSets.free_space_from_world(
            world, search_space=link_searchspace, bloat_obstacles=0.01,
        )

        target_node = free_space.node_of_point(Point3(target_position.x, target_position.y, target_position.z + 0.2,
                                                      reference_frame=self.world.root))

        all_paths = rx.dijkstra_shortest_paths(free_space.graph, free_space.box_to_index_map[target_node])
        free_space_graph = free_space.create_subgraph([path for path in all_paths])

        room_condition = self._calculate_room_event(
            world, free_space_graph, target_position
        )

        reachable_space_condition = Event(
            *[node.simple_event for node in free_space_graph.graph.nodes()]
        )
        navigation_space_condition = Event(
            *[node.simple_event for node in free_space_nav.graph.nodes()]
        )

        return reachable_space_condition, navigation_space_condition, room_condition

    def _create_navigation_circuit(
            self, target_position: Vector3
    ) -> ProbabilisticCircuit:
        """
        Creates a probabilistic circuit that samples navigation poses around the target position.
        """
        navigation_circuit = ProbabilisticCircuit()
        circuit_root = SumUnit(probabilistic_circuit=navigation_circuit)

        scale = 1.0

        p_point_root = ProductUnit(probabilistic_circuit=navigation_circuit)
        circuit_root.add_subcircuit(p_point_root, 1.0)
        target_x_p = DiracDeltaDistribution(self.target_x, target_position.x, 1.0)
        target_y_p = DiracDeltaDistribution(self.target_y, target_position.y, 1.0)

        nav_x_p = GaussianDistribution(SpatialVariables.x.value, target_position.x, scale)
        nav_y_p = GaussianDistribution(SpatialVariables.y.value, target_position.y, scale)

        p_point_root.add_subcircuit(leaf(target_x_p, navigation_circuit))
        p_point_root.add_subcircuit(leaf(target_y_p, navigation_circuit))
        p_point_root.add_subcircuit(leaf(nav_x_p, navigation_circuit))
        p_point_root.add_subcircuit(leaf(nav_y_p, navigation_circuit))

        navigation_circuit.normalize()
        return navigation_circuit

    def __iter__(self) -> Iterator[PoseStamped]:
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of ProbabilisticSemanticLocation.Location with the found valid position of the Costmap.
        """
        for params in self.generate_permutations():
            params_box = Box(params)
            target = (
                deepcopy(params_box.target)
                if isinstance(params_box.target, PoseStamped)
                else params_box.target
            )
            target_pose: PoseStamped = (
                target if isinstance(target, PoseStamped) else PoseStamped.from_spatial_type(target.global_pose)
            )
            target_position: Vector3 = target_pose.position

            self.test_world = deepcopy(self.world)
            self.test_world.name = "Test World"

            if params_box.visible_for or params_box.reachable_for:
                robot_object = (
                    params_box.visible_for
                    if params_box.visible_for
                    else params_box.reachable_for
                )
                test_robot = robot_object.from_world(self.test_world)

            # Calculate the conditions for the free space around the target position and combine them into one event
            reachable_space_condition, navigation_space_condition, room_condition = (
                self._create_free_space_conditions(self.world, target_position)
            )

            stand_on_ground = SimpleEvent(
                {SpatialVariables.z.value: (0, 0.05)}
            ).as_composite_set()
            stand_on_ground.fill_missing_variables(
                [SpatialVariables.x.value, SpatialVariables.y.value]
            )

            reachable_space_condition &= stand_on_ground
            reachable_space_condition &= room_condition
            reachable_space_condition &= navigation_space_condition

            reachable_space_condition.fill_missing_variables(
                [self.target_x, self.target_y]
            )

            # Create the circuit that samples navigation poses around the target position
            navigation_circuit = self._create_navigation_circuit(target_position)
            navigation_circuit_conditioned, navigation_circuit_p_condition = (
                navigation_circuit.truncated(reachable_space_condition)
            )

            if navigation_circuit_conditioned is None:
                continue

            samples = navigation_circuit_conditioned.sample(
                params_box.number_of_samples
            )

            samples = samples[
                np.argsort(navigation_circuit_conditioned.log_likelihood(samples))[
                    ::-1
                ]
            ]

            while len(samples) > 0:
                if isinstance(target, Body):
                    target = self.test_world.get_body_by_name(target.name)

                # Unpack the first sample, and cut out all surrounding samples that are within the costmap resolution
                _, _, nav_x, nav_y = samples[0]
                event = SimpleEvent(
                    {
                        SpatialVariables.x.value: closed(
                            nav_x - params_box.costmap_resolution,
                            nav_x + params_box.costmap_resolution,
                        ),
                        SpatialVariables.y.value: closed(
                            nav_y - params_box.costmap_resolution,
                            nav_y + params_box.costmap_resolution,
                        ),
                    }
                )
                samples = np.array(
                    [s for s in samples if not event.contains((s[2], s[3]))]
                )

                nav_quat = OrientationGenerator.generate_origin_orientation(
                    [nav_x, nav_y], target_pose
                )
                pose_candidate = PoseStamped.from_list(self.world.root,
                                                       [nav_x, nav_y, 0], nav_quat
                                                       )
                test_robot.root.parent_connection.origin = pose_candidate.to_spatial_type()
                # test_robot.set_pose(pose_candidate)

                try:
                    collision_check(test_robot, params_box.ignore_collision_with, self.test_world)
                except RobotInCollision:
                    continue

                if not (params_box.reachable_for or params_box.visible_for):
                    yield pose_candidate
                    continue

                if params_box.visible_for and not visibility_validator(
                        test_robot, target, self.test_world
                ):
                    continue

                if not params_box.reachable_for:
                    yield pose_candidate
                    continue

                grasp_descriptions = (
                    [params_box.grasp_descriptions]
                    if params_box.grasp_descriptions
                    else GraspDescription.calculate_grasp_descriptions(test_robot, target_pose)
                )

                for grasp_desc in grasp_descriptions:

                    target_sequence = _create_target_sequence(
                        grasp_desc,
                        target,
                        test_robot,
                        params_box.object_in_hand,
                        params_box.reachable_arm,
                        self.test_world,
                        rotation_agnostic=params_box.rotation_agnostic,
                    )
                    ee = ViewManager.get_arm_view(params_box.reachable_arm, test_robot)
                    is_reachable = pose_sequence_reachability_validator(
                        self.test_world.root,
                        ee.manipulator.tool_frame,
                        target_sequence,
                        self.test_world,
                        allowed_collision=params_box.ignore_collision_with,
                    )
                    if is_reachable:
                        print(f"Succeeded costmap with pose {pose_candidate}")
                        yield GraspPose(
                            pose_candidate.pose,
                            pose_candidate.header,
                            arm=params_box.reachable_arm,
                            grasp_description=grasp_desc,
                        )
