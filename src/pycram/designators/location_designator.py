import dataclasses

from typing_extensions import List, Union, Iterable, Optional, Callable

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap
from ..datastructures.enums import JointType, Arms
from ..datastructures.pose import Pose
from ..datastructures.world import World, UseProspectionWorld
from ..designator import DesignatorError, LocationDesignatorDescription
from ..local_transformer import LocalTransformer
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, reachability_validator
from ..robot_description import RobotDescription
from ..ros.logging import logdebug
from ..world_concepts.world_object import Object
from ..world_reasoning import link_pose_for_joint_config, contact


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


class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arms: List[Arms]
        """
        List of arms with which the pose can be reached, is only used when the 'rechable_for' parameter is used
        """

    def __init__(self, target: Union[Pose, ObjectDesignatorDescription.Object],
                 reachable_for: Optional[ObjectDesignatorDescription.Object] = None,
                 visible_for: Optional[ObjectDesignatorDescription.Object] = None,
                 reachable_arm: Optional[Arms] = None,
                 prepose_distance: float = 0.03,
                 check_collision_at_start: bool = True,
                 ignore_collision_with: Optional[List[Object]] = None):
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
        """
        super().__init__()
        self.target: Union[Pose, ObjectDesignatorDescription.Object] = target
        self.reachable_for: ObjectDesignatorDescription.Object = reachable_for
        self.visible_for: ObjectDesignatorDescription.Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.prepose_distance = prepose_distance
        self.check_collision_at_start = check_collision_at_start
        self.ignore_collision_with = ignore_collision_with if ignore_collision_with is not None else []

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def check_for_collision(self, robot: Object, pose: Pose) -> bool:
        """
        Check if the robot collides with any object in the world at the given pose.

        :param robot: The robot object
        :param pose: The pose to check for collision
        :return: True if the robot collides with any object, False otherwise
        """
        robot.set_pose(pose)
        floor = robot.world.get_object_by_name("floor")
        ignore = [o.name for o in self.ignore_collision_with]
        for obj in robot.world.objects:
            if obj.name in ([robot.name, floor.name] + ignore):
                continue
            if contact(robot, obj):
                logdebug(f"Robot is in contact with {obj.name} in prospection: {obj.world.is_prospection_world}"
                         f"at position {pose.position_as_list()} and z_angle {pose.z_angle}")
                return True
            logdebug(f"Robot is not in contact with {obj.name} in prospection: {obj.world.is_prospection_world}"
                     f"at position {pose.position_as_list()} and z_angle {pose.z_angle}")
        return False

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
        min_height = RobotDescription.current_robot_description.get_default_camera().minimal_height
        max_height = RobotDescription.current_robot_description.get_default_camera().maximal_height
        # This ensures that the costmaps always get a position as their origin.
        if isinstance(self.target, ObjectDesignatorDescription.Object):
            target_pose = self.target.world_object.get_pose()
        else:
            target_pose = self.target.copy()

        # ground_pose = [[target_pose[0][0], target_pose[0][1], 0], target_pose[1]]
        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
        final_map = occupancy

        if self.reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
            final_map += gaussian
        if self.visible_for:
            visible = VisibilityCostmap(min_height, max_height, 200, 0.02, Pose(target_pose.position_as_list()))
            final_map += visible

        test_robot = None
        if self.visible_for or self.reachable_for:
            robot_object = self.visible_for.world_object if self.visible_for else self.reachable_for.world_object
            test_robot = World.current_world.get_prospection_object_for_object(robot_object)
        self.ignore_collision_with = [World.current_world.get_prospection_object_for_object(o) for o in
                                      self.ignore_collision_with]
        with UseProspectionWorld():
            for maybe_pose in PoseGenerator(final_map, number_of_samples=600):
                if self.check_collision_at_start and (test_robot is not None):
                    if self.check_for_collision(test_robot, maybe_pose):
                        continue
                res = True
                arms = None
                if self.visible_for:
                    visible_prospection_object = World.current_world.get_prospection_object_for_object(self.target.world_object)
                    res = res and visibility_validator(maybe_pose, test_robot, visible_prospection_object,
                                                       World.current_world)
                if self.reachable_for:
                    hand_links = []
                    if self.reachable_arm is not None:
                        hand_links = RobotDescription.current_robot_description.get_arm_chain(
                            self.reachable_arm).end_effector.links
                    else:
                        for description in RobotDescription.current_robot_description.get_manipulator_chains():
                            hand_links += description.end_effector.links
                    allowed_collision = {test_robot: hand_links}
                    allowed_collision.update({o: o.link_names for o in self.ignore_collision_with})
                    valid, arms = reachability_validator(maybe_pose, test_robot, target_pose,
                                                         allowed_collision=allowed_collision,
                                                         arm=self.reachable_arm,
                                                         prepose_distance=self.prepose_distance)
                    if self.reachable_arm:
                        res = res and valid and self.reachable_arm in arms
                    else:
                        res = res and valid
                if res:
                    yield self.Location(maybe_pose, arms)


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        arms: List[Arms]
        """
        List of arms that can be used to for accessing from this pose
        """

    def __init__(self, handle_desig: ObjectPart.Object, robot_desig: ObjectDesignatorDescription.Object):
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

    def __iter__(self) -> Location:
        """
        Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
        handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
        the handle can be grasped if the drawer is open.

        :yield: A location designator containing the pose and the arms that can be used.
        """
        # ground_pose = [[self.handle.part_pose[0][0], self.handle.part_pose[0][1], 0], self.handle.part_pose[1]]
        ground_pose = Pose(self.handle.part_pose.position_as_list())
        ground_pose.position.z = 0
        occupancy = OccupancyCostmap(distance_to_obstacle=0.25, from_ros=False, size=200, resolution=0.02,
                                     origin=ground_pose)
        gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)

        final_map = occupancy + gaussian

        test_robot = World.current_world.get_prospection_object_for_object(self.robot)

        # Find a Joint of type prismatic which is above the handle in the URDF tree
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

        with UseProspectionWorld():
            for maybe_pose in PoseGenerator(final_map, number_of_samples=600,
                                             orientation_generator=lambda p, o: PoseGenerator.generate_orientation(p, half_pose)):

                hand_links = []
                for description in RobotDescription.current_robot_description.get_manipulator_chains():
                    hand_links += description.links

                valid_init, arms_init = reachability_validator(maybe_pose, test_robot, init_pose,
                                                               allowed_collision={test_robot: hand_links})

                valid_goal, arms_goal = reachability_validator(maybe_pose, test_robot, goal_pose,
                                                               allowed_collision={test_robot: hand_links})

                arms_list = list(set(arms_init).intersection(set(arms_goal)))

                if valid_init and valid_goal and len(arms_list) > 0:
                    yield self.Location(maybe_pose, arms_list)


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, urdf_link_name, part_of, for_object=None):
        """
        Creates a distribution over a urdf link to sample poses which are on this link. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        :param urdf_link_name: Name of the urdf link for which a distribution should be calculated
        :param part_of: Object of which the urdf link is a part
        :param for_object: Optional object that should be placed at the found location
        """
        super().__init__()
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
