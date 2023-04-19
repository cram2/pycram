import dataclasses

from .object_designator import ObjectDesignatorDescription
from ..bullet_world import Object, BulletWorld
from ..designator import Designator, DesignatorError, DesignatorDescription, ResolutionError
from ..process_module import ProcessModule
from ..orm.base import Quaternion, Position, Base
from ..robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from typing import Tuple, List, Dict, get_type_hints
import sqlalchemy.orm


class MotionDesignatorDescription(DesignatorDescription):
    class Motion:
        cmd: str

        def perform(self):
            return ProcessModule.perform(self)

    def ground(self) -> Motion:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")

    def to_sql(self) -> Base:
        """
        Create an ORM object that corresponds to this description.
        :return: The created ORM object.
        """
        raise NotImplementedError(f"{type(self)} has no implementation of to_sql. Feel free to implement it.")

    def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
        """
        Add and commit this and all related objects to the session.
        Auto-Incrementing primary keys and foreign keys have to be filled by this method.

        :param session: Session with a database that is used to add and commit the objects
        :param args: Possible extra arguments
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM object
        """
        raise NotImplementedError(f"{type(self)} has no implementation of insert. Feel free to implement it.")

    def get_slots(self):
        """
        Returns a list of all slots of this description. Can be used for inspecting
        different descriptions and debugging.
        :return: A list of all slots.
        """
        return list(self.__dict__.keys()).remove('cmd')

    def _check_properties(self, desig: str, exclude: List[str] = []) -> None:
        """
        Checks the properties of this description. It will be checked if any attribute is
        None and if any attribute has to wrong type according to the type hints in
        the description class.
        It is possible to provide a list of attributes which should not be checked.

        :param desig: The current type of designator, will be used when raising an
                        Exception as output.
        :param exclude: A list of properties which should not be checked.
        """
        right_types = get_type_hints(self.Motion)
        attributes = self.__dict__.copy()
        del attributes["resolve"]
        missing = []
        wrong_type = {}
        current_type = {}
        for k in attributes.keys():
            if attributes[k] == None and not attributes[k] in exclude:
                missing.append(k)
            elif type(attributes[k]) != right_types[k] and not attributes[k] in exclude:
                wrong_type[k] = right_types[k]
                current_type[k] = type(attributes[k])
        if missing != [] or wrong_type != {}:
            raise ResolutionError(missing, wrong_type, current_type, desig)


class MoveMotion(MotionDesignatorDescription):
    """
    Definition of types. Is used in _check_missing_properties for evaluating
    the types of given properties.
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        target: list

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, target, resolver=None):
        """
        Navigates to robot to the given target
        :param target: Position and Orientation of the navigation goal
        :param resolver: A method with which to resolve the description
        """
        super().__init__(resolver)
        self.cmd = "navigate"
        self.target = target

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.target)


class PickUpMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        object_desig: Object
        arm: str
        grasp: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, object_desig, grasp=None, arm=None, resolver=None):
        super().__init__(resolver)
        self.cmd: str = 'pick-up'
        self.object_desig = object_desig
        self.arm: str = arm
        self.grasp: str = grasp

    def ground(self):
        arm = "left" if not self.arm else self.arm
        grasp = "left" if not self.grasp else self.grasp
        return self.Motion(self.cmd, self.object_desig, arm, grasp)


class PlaceMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        object: ObjectDesignatorDescription.Object
        target: Tuple[List[float], List[float]]
        arm: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, object_desig, target, arm=None, resolver=None):
        super().__init__(resolver)
        self.cmd: str = 'place'
        self.object_desig = object_desig
        self.target: Tuple[List[float], List[float]] = target
        self.arm: str = arm

    def ground(self) -> Motion:
        arm = "left" if not self.arm else self.arm
        return self.Motion(self.cmd, self.object_desig, self.target, arm)


class AccessingMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        drawer_joint: str
        drawer_handle: str
        part_of: Object
        arm: str
        distance: float

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, drawer_joint, drawer_handle, part_of, arm=None, distance=0.3, resolver=None):
        super().__init__(resolver)
        self.cmd: str = 'access'
        self.drawer_joint = drawer_joint
        self.drawer_handle = drawer_handle
        self.part_of = part_of
        self.arm = arm
        self.distance = distance
        self.gripper = None

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.drawer_joint, self.drawer_handle, self.part_of, self.arm, self.distance)


class MoveTCPMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        target: list
        arm: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, target, arm=None, resolver=None):
        super().__init__(resolver)
        self.cmd: str = 'move-tcp'
        self.target: Tuple[List[float], List[float]] = target
        self.arm: str = arm

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.target, self.arm)


class LookingMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        target: Tuple[List[float], List[float]]

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, target=None, object=None, resolver=None):
        """

        :param target: Position and orientation of the target
        :param object: An Object in the BulletWorld
        :param resolver:
        """
        super().__init__(resolver)
        self.cmd: str = 'looking'
        self.target: Tuple[List[float], List[float]] = target
        self.object: Object = object

    def ground(self) -> Motion:
        if not self.target and self.object:
            self.target = self.object.get_position_and_orientation()
        if len(self.target) == 3:
            self.target = [self.target, [0, 0, 0, 1]]
        return self.Motion(self.cmd, self.target)


class MoveGripperMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        motion: str
        gripper: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, motion, gripper, resolver=None):
        super().__init__(resolver)
        self.cmd: str = 'move-gripper'
        self.motion: str = motion
        self.gripper: str = gripper

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.motion, self.gripper)


class DetectingMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        object_type: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, object_type, resolver=None):
        super().__init__(resolver)
        self.cmd = 'detecting'
        self.object_type = object_type

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.object_type)


class MoveArmJointsMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        left_arm_poses: Dict[str, float]
        right_arm_poses: Dict[str, float]

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, left_arm_config=None, right_arm_config=None, left_arm_poses=None, right_arm_poses=None,
                 resolver=None):
        super().__init__(resolver)
        self.cmd = 'move-arm-joints'
        self.left_arm_config: str = left_arm_config
        self.right_arm_config: str = right_arm_config
        self.left_arm_poses: Dict[str, float] = left_arm_poses
        self.right_arm_poses: Dict[str, float] = right_arm_poses

    def ground(self) -> Motion:
        if self.left_arm_poses:
            left_poses = self.left_arm_poses
        else:
            left_poses = robot_description.i.get_static_joint_chain("left", self.left_arm_config)

        if self.right_arm_poses:
            right_poses = self.right_arm_poses
        else:
            right_poses = robot_description.i.get_static_joint_chain("right", self.right_arm_config)
        return self.Motion(self.cmd, left_poses, right_poses)


class WorldStateDetectingMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        object_type: str

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, object_type, resolver=None):
        super().__init__(resolver)
        self.cmd = 'world-state-detecting'
        self.object_type: str = object_type

    def ground(self) -> Motion:
        return self.Motion(self.cmd, self.object_type)


class MoveJointsMotion(MotionDesignatorDescription):
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        names: list
        positions: list

        def perform(self):
            return ProcessModule.perform(self)

    def __init__(self, names, positions, resolver=None):
        super().__init__(resolver)
        self.cmd = "move-joints"
        self.names = names
        self.positions = positions

    def ground(self) -> Motion:
        if len(self.names) != len(self.positions):
            raise DesignatorError("[Motion Designator][Move Joints] The length of names and positions does not match")
        for i in range(len(self.names)):
            lower, upper = BulletWorld.robot.get_joint_limits(self.names[i])
            if self.positions[i] < lower or self.positions[i] > upper:
                raise DesignatorError(
                    f"[Motion Designator][Move Joints] The given configuration for the Joint {self.names[i]} violates its limits")
        return self.Motion(self.cmd, self.names, self.positions)
