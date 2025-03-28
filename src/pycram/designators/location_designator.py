import dataclasses

import numpy as np
from typing_extensions import List, Union, Iterable, Optional, Callable

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, Costmap
from ..datastructures.enums import JointType, Arms, Grasp
from ..datastructures.pose import Pose, GraspDescription
from ..datastructures.world import World, UseProspectionWorld
from ..designator import DesignatorError, LocationDesignatorDescription
from ..failures import RobotInCollision
from ..local_transformer import LocalTransformer
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, \
    pose_sequence_reachability_validator, collision_check
from ..world_concepts.world_object import Object
from ..world_reasoning import link_pose_for_joint_config


class Location(LocationDesignatorDescription):
    """
    Default location designator which only wraps a pose.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, pose: Pose):
        """
        Basic location designator that represents a single pose.

        :param pose: The pose that should be represented by this location designator
        """
        super().__init__()
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

    def __init__(self, relative_pose: Pose = None, reference_object: ObjectDesignatorDescription = None):
        """
        Location designator representing a location relative to a given object.

        :param relative_pose: Pose that should be relative, in world coordinate frame
        :param reference_object: Object to which the pose should be relative
        """
        super().__init__()
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


@dataclasses.dataclass
class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arm: Optional[Arms]
        """
        List of arms with which the pose can be reached, is only used when the 'reachable_for' parameter is used
        """

        grasp_description: Optional[GraspDescription]
        """
        The grasp configuration used to reach the pose
        """

    target: Union[Pose, ObjectDesignatorDescription.Object]
    """
    Target pose or object for which the pose should be reachable
    """

    reachable_for: Optional[ObjectDesignatorDescription.Object] = None
    """
    Robot for which the pose should be reachable
    """

    visible_for: Optional[ObjectDesignatorDescription.Object] = None
    """
    Robot for which the pose should be visible
    """

    reachable_arms: Optional[List[Arms]] = dataclasses.field(default_factory=lambda: [Arms.LEFT, Arms.RIGHT])
    """
    List of arms which should be tried out
    """

    ignore_collision_with: Optional[List[Object]] = dataclasses.field(default_factory=list)
    """
    List of objects with which the robot should not collide
    """

    grasp_descriptions: Optional[List[GraspDescription]] = None
    """
    List of grasp descriptions that should be tried out
    """

    object_in_hand: Optional[ObjectDesignatorDescription.Object] = None
    """
    Object that is currently in the hand of the robot, if 
    """

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def setup_costmaps(self, target: Pose, robot: Object) -> Costmap:
        """
        Sets up the costmaps for the given target and robot. The costmaps are merged and stored in the final_map
        """
        target_pose = Pose([target.position.x, target.position.y, target.position.z],
                           [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
        final_map = occupancy

        if self.visible_for:
            camera = robot.robot_description.get_default_camera()
            visible = VisibilityCostmap(camera.minimal_height, camera.maximal_height, 200, 0.02,
                                        Pose(target_pose.position_as_list()), target_object=target, robot=robot)
            final_map += visible

        if self.reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
            final_map += gaussian

        return final_map

    def __iter__(self):
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
        reachable_arms = self.reachable_arms if self.reachable_arms else [Arms.LEFT, Arms.RIGHT]

        target = self.target.copy() if isinstance(self.target, Pose) else self.target.world_object

        if (self.reachable_for or self.visible_for):
            robot_object = (self.visible_for or self.reachable_for).world_object
        else:
            robot_object = World.robot
        test_robot = World.current_world.get_prospection_object_for_object(robot_object)

        ignore_collision_with = [World.current_world.get_prospection_object_for_object(obj.world_object)
                                 for obj in self.ignore_collision_with]

        allowed_collision = {object: object.link_id_to_name[-1] for object in ignore_collision_with}
        if self.object_in_hand:
            prospection_object = World.current_world.get_prospection_object_for_object(self.object_in_hand.world_object)
            allowed_collision.update({prospection_object: prospection_object.link_id_to_name[-1]})

        final_map = self.setup_costmaps(target, test_robot)

        with (UseProspectionWorld()):
            for pose_candidate in PoseGenerator(final_map, number_of_samples=600):
                pose_candidate.position.z = 0
                test_robot.set_pose(pose_candidate)
                try:
                    collision_check(test_robot, allowed_collision)
                except RobotInCollision:
                    continue

                if not (self.reachable_for or self.visible_for):
                    yield self.Location(pose_candidate, None, None)
                    continue

                if self.visible_for and not visibility_validator(test_robot, target):
                    continue

                if not self.reachable_for:
                    yield self.Location(pose_candidate, None, None)
                    continue

                grasp_descriptions = self.grasp_descriptions or target.calculate_grasp_descriptions(test_robot)
                for grasp_description in grasp_descriptions:
                    for arm in reachable_arms:
                        end_effector = test_robot.robot_description.get_arm_chain(arm).end_effector
                        grasp_quaternion = end_effector.grasps[grasp_description]
                        approach_axis = end_effector.get_approach_axis()
                        if self.object_in_hand:
                            current_target_pose = self.object_in_hand.world_object.attachments[
                                World.robot].get_child_link_target_pose_given_parent(target)
                            approach_offset_cm = self.object_in_hand.world_object.get_approach_offset()
                        else:
                            current_target_pose = target.copy() if isinstance(target, Pose) else \
                                target.get_grasp_pose(end_effector, grasp_description)
                            current_target_pose.rotate_by_quaternion(grasp_quaternion)
                            approach_offset_cm = 0.1 if isinstance(target, Pose) else target.get_approach_offset()

                        lift_pose = current_target_pose.copy()
                        lift_pose.position.z += 0.1

                        retract_pose = LocalTransformer().translate_pose_along_local_axis(current_target_pose,
                                                                                          approach_axis,
                                                                                          -approach_offset_cm)

                        target_sequence = ([lift_pose, current_target_pose, retract_pose] if self.object_in_hand
                                           else [retract_pose, current_target_pose, lift_pose])

                        is_reachable = pose_sequence_reachability_validator(test_robot, target_sequence,
                                                                            arm=arm,
                                                                            allowed_collision=allowed_collision)
                        if is_reachable:
                            yield self.Location(pose_candidate, arm, grasp_description)


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        arm: Arms
        """
        Arm that can be used to for accessing from this pose
        """

    def __init__(self, handle_desig: ObjectPart.Object,
                 robot_desig: ObjectDesignatorDescription.Object):
        """
        Describes a position from where a drawer can be opened. For now this position should be calculated before the
        drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

        :param handle_desig: ObjectPart designator for handle of the drawer
        :param robot_desig: Object designator for the robot which should open the drawer
        """
        super().__init__()
        self.handle: ObjectPart.Object = handle_desig
        self.robot: ObjectDesignatorDescription.Object = robot_desig.world_object

    def ground(self) -> Location:
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

    def setup_costmaps(self) -> Costmap:
        """
        Sets up the costmaps for the given handle and robot. The costmaps are merged and stored in the final_map.
        """
        ground_pose = Pose(self.handle.part_pose.position_as_list())
        ground_pose.position.z = 0
        occupancy = OccupancyCostmap(distance_to_obstacle=0.25, from_ros=False, size=200, resolution=0.02,
                                     origin=ground_pose)
        final_map = occupancy

        gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
        final_map += gaussian

        return final_map

    def __iter__(self) -> Location:
        """
        Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
        handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
        the handle can be grasped if the drawer is open.

        :yield: A location designator containing the pose and the arms that can be used.
        """

        # Find a Joint that moves with the handle in the URDF tree
        container_joint = self.handle.world_object.find_joint_above_link(self.handle.name)

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

        joint_type = self.handle.world_object.joints[container_joint].type
        final_map = self.setup_costmaps()
        if joint_type == JointType.PRISMATIC:
            self.adjust_map_for_drawer_opening(final_map, init_pose, goal_pose)

        target_sequence = [init_pose, half_pose, goal_pose]
        test_robot = World.current_world.get_prospection_object_for_object(self.robot)

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
                        yield self.Location(pose_candidate, arm_chain.arm_type)


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

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
        self.link_name: str = link_name
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: Optional[ObjectDesignatorDescription.Object] = for_object
        self.edges_only: bool = edges_only
        self.horizontal_edges_only: bool = horizontal_edges_only
        self.edge_size_in_meters: float = edge_size_in_meters
        self.sem_costmap: Optional[SemanticCostmap] = None

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
        self.sem_costmap = SemanticCostmap(self.part_of.world_object, self.link_name)
        if self.edges_only or self.horizontal_edges_only:
            self.sem_costmap = self.sem_costmap.get_edges_map(self.edge_size_in_meters,
                                                              horizontal_only=self.horizontal_edges_only)
        height_offset = 0
        if self.for_object:
            min_p, max_p = self.for_object.world_object.get_axis_aligned_bounding_box().get_min_max_points()
            height_offset = (max_p.z - min_p.z) / 2
        for maybe_pose in PoseGenerator(self.sem_costmap):
            maybe_pose.position.z += height_offset
            yield self.Location(maybe_pose)
