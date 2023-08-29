import dataclasses
import time
from typing import List, Tuple, Union, Iterable, Optional, Callable

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..bullet_world import Object, BulletWorld, Use_shadow_world
from ..bullet_world_reasoning import link_pose_for_joint_config
from ..designator import Designator, DesignatorError, LocationDesignatorDescription
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap
from ..robot_descriptions import robot_description
from ..enums import JointType
from ..helper import transform
from ..plan_failures import EnvironmentManipulationImpossible
from ..pose_generator_and_validator import pose_generator, visibility_validator, reachability_validator, \
    generate_orientation
from ..robot_description import ManipulatorDescription
from ..pose import Pose


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
        :param resolver: An alternative resolver that returns a resolved location
        """
        super().__init__(resolver)
        self.pose: Pose = pose

    def ground(self) -> Location:
        """
        Default resolver which returns a resolved designator which contains the pose given in init.

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
        :param resolver: An alternative resolver that returns a resolved location for the input parameter
        """
        super().__init__(resolver)
        self.relative_pose: Pose = relative_pose
        self.reference_object: ObjectDesignatorDescription = reference_object

    def ground(self) -> Location:
        """
        Default resolver which returns a resolved location for description input. Resolved location is the first result
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
        obj_pose_world = obj_grounded.get_position_and_location()
        obj_pose_world_flat = [i for sublist in obj_pose_world for i in sublist]
        relative_pose_flat = [i for sublist in self.relative_pose for i in sublist]
        pose = transform(obj_pose_world_flat, relative_pose_flat, local_coords=False)

        yield self.Location(self.relative_pose, pose, self.reference_object)


class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arms: List[str]
        """
        List of arms with which the pose can be reached, is only used when the 'rechable_for' parameter is used
        """

    def __init__(self, target: Union[Pose, ObjectDesignatorDescription.Object],
                 reachable_for: Optional[ObjectDesignatorDescription.Object] = None,
                 visible_for: Optional[ObjectDesignatorDescription.Object] = None,
                 reachable_arm: Optional[str] = None, resolver: Optional[Callable] = None):
        """
        Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
        visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

        :param target: Location for which visibility or reachability should be calculated
        :param reachable_for: Object for which the reachability should be calculated, usually a robot
        :param visible_for: Object for which the visibility should be calculated, usually a robot
        :param reachable_arm: An optional arm with which the target should be reached
        :param resolver: An alternative resolver that returns a resolved location for the given input of this description
        """
        super().__init__(resolver)
        self.target: Union[Pose, ObjectDesignatorDescription.Object] = target
        self.reachable_for: ObjectDesignatorDescription.Object = reachable_for
        self.visible_for: ObjectDesignatorDescription.Object = visible_for
        self.reachable_arm: Optional[str] = reachable_arm

    def ground(self) -> Location:
        """
        Default resolver which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

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
        min_height = list(robot_description.cameras.values())[0].min_height
        max_height = list(robot_description.cameras.values())[0].max_height
        # This ensures that the costmaps always get a position as their origin.
        if isinstance(self.target, ObjectDesignatorDescription.Object):
            target_pose = self.target.bullet_world_object.get_pose()
        else:
            target_pose = self.target.copy()

        # ground_pose = [[target_pose[0][0], target_pose[0][1], 0], target_pose[1]]
        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.4, False, 200, 0.02, ground_pose)
        final_map = occupancy

        if self.reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
            final_map += gaussian
        if self.visible_for:
            visible = VisibilityCostmap(min_height, max_height, 200, 0.02, Pose(target_pose.position_as_list()))
            final_map += visible

        if self.visible_for or self.reachable_for:
            robot_object = self.visible_for.bullet_world_object if self.visible_for else self.reachable_for.bullet_world_object
            test_robot = BulletWorld.current_bullet_world.get_shadow_object(robot_object)

        with Use_shadow_world():

            for maybe_pose in pose_generator(final_map, number_of_samples=600):
                res = True
                arms = None
                if self.visible_for:
                    res = res and visibility_validator(maybe_pose, test_robot, target_pose,
                                                       BulletWorld.current_bullet_world)
                if self.reachable_for:
                    hand_links = []
                    for name, chain in robot_description.chains.items():
                        if isinstance(chain, ManipulatorDescription):
                            hand_links += chain.gripper.links
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
    Location designator which describes poses used for opening drawers
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        arms: List[str]
        """
        List of arms that can be used to for accessing from this pose
        """

    def __init__(self, handle_desig: ObjectPart.Object, robot_desig: ObjectDesignatorDescription.Object, resolver=None):
        """
        Describes a position from where a drawer can be opened. For now this position should be calculated before the
        drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

        :param handle_desig: ObjectPart designator for handle of the drawer
        :param robot: Object designator for the robot which should open the drawer
        :param resolver: An alternative resolver to create the location
        """
        super().__init__(resolver)
        self.handle: ObjectPart.Object = handle_desig
        self.robot: ObjectDesignatorDescription.Object = robot_desig.bullet_world_object

    def ground(self) -> Location:
        """
        Default resolver for this location designator, just returns the first element from the iteration

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
        occupancy = OccupancyCostmap(distance_to_obstacle=0.4, from_ros=False, size=200, resolution=0.02,
                                     origin=ground_pose)
        gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)

        final_map = occupancy + gaussian

        test_robot = BulletWorld.current_bullet_world.get_shadow_object(self.robot)

        # Find a Joint of type prismatic which is above the handle in the URDF tree
        container_joint = self.handle.bullet_world_object.find_joint_above(self.handle.name, JointType.PRISMATIC)

        init_pose = link_pose_for_joint_config(self.handle.bullet_world_object, {
            container_joint: self.handle.bullet_world_object.get_joint_limits(container_joint)[0]},
                                               self.handle.name)

        # Calculate the pose the handle would be in if the drawer was to be fully opened
        goal_pose = link_pose_for_joint_config(self.handle.bullet_world_object, {
            container_joint: self.handle.bullet_world_object.get_joint_limits(container_joint)[1] - 0.05},
                                               self.handle.name)

        # Handle position for calculating rotation of the final pose
        half_pose = link_pose_for_joint_config(self.handle.bullet_world_object, {
            container_joint: self.handle.bullet_world_object.get_joint_limits(container_joint)[1] / 1.5},
                                               self.handle.name)

        with Use_shadow_world():
            for maybe_pose in pose_generator(final_map, number_of_samples=600,
                                             orientation_generator=lambda p, o: generate_orientation(p, half_pose)):

                hand_links = []
                for name, chain in robot_description.chains.items():
                    if isinstance(chain, ManipulatorDescription):
                        hand_links += chain.gripper.links

                valid_init, arms_init = reachability_validator(maybe_pose, test_robot, init_pose,
                                                               allowed_collision={test_robot: hand_links})

                valid_goal, arms_goal = reachability_validator(maybe_pose, test_robot, goal_pose,
                                                               allowed_collision={test_robot: hand_links})

                if valid_init and valid_goal:
                    yield self.Location(maybe_pose, list(set(arms_init).intersection(set(arms_goal))))


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
        :param resolver: An alternative resolver that creates a resolved location for the input parameter of this description
        """
        super().__init__(resolver)
        self.urdf_link_name: str = urdf_link_name
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: Optional[ObjectDesignatorDescription.Object] = for_object

    def ground(self) -> Location:
        """
        Default resolver which returns the first element of the iterator of this instance.

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
        sem_costmap = SemanticCostmap(self.part_of.bullet_world_object, self.urdf_link_name)
        height_offset = 0
        if self.for_object:
            min, max = self.for_object.bullet_world_object.get_AABB()
            height_offset = (max[2] - min[2]) / 2
        for maybe_pose in pose_generator(sem_costmap):
            maybe_pose.position.z += height_offset
            yield self.Location(maybe_pose)
