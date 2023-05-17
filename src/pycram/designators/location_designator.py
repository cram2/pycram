import dataclasses
from typing import List, Tuple, Union, Iterable

from .object_designator import ObjectDesignatorDescription
from ..bullet_world import Object, BulletWorld, Use_shadow_world
from ..designator import Designator, DesignatorError, DesignatorDescription
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from ..helper import transform
from ..pose_generator_and_validator import pose_generator, visibility_validator, reachability_validator


class LocationDesignatorDescription(DesignatorDescription):
    @dataclasses.dataclass
    class Location:
        pose: Tuple[List[float], List[float]]

    def __init__(self, resolver=None):
        super().__init__(resolver)

    def ground(self) -> Location:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


class Location(LocationDesignatorDescription):
    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, pose, resolver=None):
        super().__init__(resolver)
        self.pose: Tuple[List[float], List[float]] = pose

    def ground(self) -> Location:
        return self.Location(self.pose)


class ObjectRelativeLocation(LocationDesignatorDescription):
    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        relative_pose: List[float]
        reference_object: ObjectDesignatorDescription.Object

    def __init__(self, relative_pose: List[float] = None, reference_object: ObjectDesignatorDescription.Object = None,
                 resolver=None):
        super().__init__(resolver)
        self.relative_pose = relative_pose
        self.reference_object = reference_object

    def ground(self) -> Location:
        return next(iter(self))

    def __iter__(self) -> Iterable[Location]:
        if self.relative_pose is None or self.reference_object is None:
            raise DesignatorError(
                "Could not ground ObjectRelativeLocation: (Relative) pose and reference object must be given")
        # Fetch the object pose and yield the grounded description
        obj_grounded = self.reference_object.reference()
        obj_pose_world = obj_grounded.get_position_and_location()
        obj_pose_world_flat = [i for sublist in obj_pose_world for i in sublist]
        relative_pose_flat = [i for sublist in self.relative_pose for i in sublist]
        pose = transform(obj_pose_world_flat, relative_pose_flat, local_coords=False)

        yield self.Location(None, pose, self.reference_object)


class CostmapLocation(LocationDesignatorDescription):
    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arms: List[str]

    def __init__(self, target, reachable_for=None, visible_for=None, reachable_arm=None, resolver=None):
        super().__init__(resolver)
        self.target: Union[Tuple[List[float], List[float]], ObjectDesignatorDescription.Object] = target
        self.reachable_for: ObjectDesignatorDescription.Object = reachable_for
        self.visible_for: ObjectDesignatorDescription.Object = visible_for
        self.reachable_arm: str = reachable_arm

    def ground(self) -> Location:
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
           pass the pose is considered valid and added to the list of valid poses.
           After a maximum of 15 valid poses are found the validation of poses stops
           and the found valid poses are yielded as solutions for the designator.

           :yield: An instance of CostmapLocation.Location with a valid position that satisfies the given constraints
           """
        min_height = list(robot_description.i.cameras.values())[0].min_height
        max_height = list(robot_description.i.cameras.values())[0].max_height
        # This ensures that the costmaps always get a position as their origin.
        if isinstance(self.target, ObjectDesignatorDescription.Object):
            target_pose = self.target.bullet_world_object.get_position_and_orientation()
        else:
            target_pose = self.target

        ground_pose = [[target_pose[0][0], target_pose[0][1], 0], target_pose[1]]

        occupancy = OccupancyCostmap(0.4, False, 200, 0.02, [ground_pose[0], [0, 0, 0, 1]],
                                     BulletWorld.current_bullet_world)
        final_map = occupancy

        if self.reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, [ground_pose[0], [0, 0, 0, 1]])
            final_map += gaussian
        if self.visible_for:
            visible = VisibilityCostmap(min_height, max_height, 200, 0.02, [target_pose[0], [0, 0, 0, 1]])
            final_map += visible

        if self.visible_for or self.reachable_for:
            robot_object = self.visible_for.bullet_world_object if self.visible_for else self.reachable_for.bullet_world_object
            test_robot = BulletWorld.current_bullet_world.get_shadow_object(robot_object)

        with Use_shadow_world():

            for maybe_pose in pose_generator(final_map):
                res = True
                arms = None
                if self.visible_for:
                    res = res and visibility_validator(maybe_pose, test_robot, target_pose,
                                                       BulletWorld.current_bullet_world)
                if self.reachable_for:
                    valid, arms = reachability_validator(maybe_pose, test_robot, target_pose,
                                                         BulletWorld.current_bullet_world)
                    if self.reachable_arm:
                        res = res and valid and self.reachable_arm in arms
                    else:
                        res = res and valid

                if res:
                    yield self.Location(list(maybe_pose), arms)


class SemanticCostmapLocation(LocationDesignatorDescription):
    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, urdf_link_name, part_of, for_object=None, resolver=None):
        super().__init__(resolver)
        self.urdf_link_name: str = urdf_link_name
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: ObjectDesignatorDescription.Object = for_object

    def ground(self) -> Location:
        return next(iter(self))

    def __iter__(self):
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instancce of SemanticCostmapLocation.Location with the found valid position of the Costmap.
        """
        sem_costmap = SemanticCostmap(self.part_of.bullet_world_object, self.urdf_link_name)
        height_offset = 0
        if self.for_object:
            min, max = self.for_object.bullet_world_object.get_AABB()
            height_offset = (max[2] - min[2]) / 2
        for maybe_pose in pose_generator(sem_costmap):
            position = [maybe_pose[0][0], maybe_pose[0][1], maybe_pose[0][2] + height_offset]
            yield self.Location([position, maybe_pose[1]])
