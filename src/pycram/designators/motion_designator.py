from abc import ABC, abstractmethod
from dataclasses import dataclass

from sqlalchemy.orm import Session
from .object_designator import ObjectDesignatorDescription, ObjectPart, RealObject
from ..designator import ResolutionError
from ..orm.base import ProcessMetaData
from ..plan_failures import PerceptionObjectNotFound
from ..process_module import ProcessModuleManager
from ..orm.motion_designator import (MoveMotion as ORMMoveMotion,
                                     MoveTCPMotion as ORMMoveTCPMotion, LookingMotion as ORMLookingMotion,
                                     MoveGripperMotion as ORMMoveGripperMotion, DetectingMotion as ORMDetectingMotion,
                                     OpeningMotion as ORMOpeningMotion, ClosingMotion as ORMClosingMotion,
                                     Motion as ORMMotionDesignator)
from ..datastructures.enums import ObjectType, Arms, GripperState

from typing_extensions import Dict, Optional, get_type_hints, get_args, get_origin
from ..datastructures.pose import Pose
from ..tasktree import with_tree


@dataclass
class BaseMotion(ABC):

    @abstractmethod
    def perform(self):
        """
        Passes this designator to the process module for execution. Will be overwritten by each motion.
        """
        pass
        # return ProcessModule.perform(self)

    @abstractmethod
    def to_sql(self) -> ORMMotionDesignator:
        """
        Create an ORM object that corresponds to this description. Will be overwritten by each motion.

        :return: The created ORM object.
        """
        return ORMMotionDesignator()

    @abstractmethod
    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        """
        Add and commit this and all related objects to the session.
        Auto-Incrementing primary keys and foreign keys have to be filled by this method.

        :param session: Session with a database that is used to add and commit the objects
        :param args: Possible extra arguments
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM motion.
        """
        metadata = ProcessMetaData().insert(session)

        motion = self.to_sql()
        motion.process_metadata = metadata

        return motion

    def __post_init__(self):
        """
        Checks if types are missing or wrong
        """
        right_types = get_type_hints(self)
        attributes = self.__dict__.copy()

        missing = []
        wrong_type = {}
        current_type = {}

        for k in attributes.keys():
            attribute = attributes[k]
            attribute_type = type(attributes[k])
            right_type = right_types[k]
            types = get_args(right_type)
            if attribute is None:
                if not any([x is type(None) for x in get_args(right_type)]):
                    missing.append(k)
            elif attribute_type is not right_type:
                if attribute_type not in types:
                    if attribute_type not in [get_origin(x) for x in types if x is not type(None)]:
                        wrong_type[k] = right_types[k]
                        current_type[k] = attribute_type
        if missing != [] or wrong_type != {}:
            raise ResolutionError(missing, wrong_type, current_type, self.__class__)


@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

    target: Pose
    """
    Location to which the robot should be moved
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.navigate().execute(self)
        # return ProcessModule.perform(self)

    def to_sql(self) -> ORMMoveMotion:
        return ORMMoveMotion()

    def insert(self, session, *args, **kwargs) -> ORMMoveMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_tcp().execute(self)

    def to_sql(self) -> ORMMoveTCPMotion:
        return ORMMoveTCPMotion(self.arm, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveTCPMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: Pose

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.looking().execute(self)

    def to_sql(self) -> ORMLookingMotion:
        return ORMLookingMotion()

    def insert(self, session: Session, *args, **kwargs) -> ORMLookingMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_gripper().execute(self)

    def to_sql(self) -> ORMMoveGripperMotion:
        return ORMMoveGripperMotion(self.motion, self.gripper, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveGripperMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot
    """

    object_type: ObjectType
    """
    Type of the object that should be detected
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        world_object = pm_manager.detecting().execute(self)
        if not world_object:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {self.object_type} in the FOV of the robot")
        if ProcessModuleManager.execution_type == "real":
            return RealObject.Object(world_object.name, world_object.obj_type,
                                                  world_object, world_object.get_pose())

        return ObjectDesignatorDescription.Object(world_object.name, world_object.obj_type,
                                                  world_object)

    def to_sql(self) -> ORMDetectingMotion:
        return ORMDetectingMotion(self.object_type)

    def insert(self, session: Session, *args, **kwargs) -> ORMDetectingMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class MoveArmJointsMotion(BaseMotion):
    """
    Moves the joints of each arm into the given position
    """

    left_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the left arm joints
    """
    right_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the right arm joints
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class WorldStateDetectingMotion(BaseMotion):
    """
    Detects an object based on the world state.
    """

    object_type: ObjectType
    """
    Object type that should be detected
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.world_state_detecting().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class MoveJointsMotion(BaseMotion):
    """
    Moves any joint on the robot
    """

    names: list
    """
    List of joint names that should be moved 
    """
    positions: list
    """
    Target positions of joints, should correspond to the list of names
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.open().execute(self)

    def to_sql(self) -> ORMOpeningMotion:
        return ORMOpeningMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMOpeningMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.close().execute(self)

    def to_sql(self) -> ORMClosingMotion:
        return ORMClosingMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMClosingMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion
