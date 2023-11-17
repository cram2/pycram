import dataclasses

from sqlalchemy.orm import Session
from .object_designator import ObjectDesignatorDescription, ObjectPart, RealObject
from ..bullet_world import Object, BulletWorld
from ..designator import DesignatorError
from ..plan_failures import PerceptionObjectNotFound
from ..process_module import ProcessModuleManager
from ..robot_descriptions import robot_description
from ..designator import MotionDesignatorDescription
from ..orm.motion_designator import (MoveMotion as ORMMoveMotion, AccessingMotion as ORMAccessingMotion,
                                     MoveTCPMotion as ORMMoveTCPMotion, LookingMotion as ORMLookingMotion,
                                     MoveGripperMotion as ORMMoveGripperMotion, DetectingMotion as ORMDetectingMotion,
                                     WorldStateDetectingMotion as ORMWorldStateDetectingMotion,
                                     OpeningMotion as ORMOpeningMotion, ClosingMotion as ORMClosingMotion)

from typing import List, Dict, Callable, Optional
from ..pose import Pose
from ..task import with_tree


class MoveMotion(MotionDesignatorDescription):
    """
    Moves the robot to a designated location
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
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
            motion.pose_id = pose.id

            session.add(motion)
            session.commit()

            return motion

    def __init__(self, target: Pose, resolver: Callable = None):
        """
        Navigates to robot to the given target

        :param target: Position and Orientation of the navigation goal
        :param resolver: A method with which to resolve the description
        """
        super().__init__(resolver)
        self.cmd: str = "navigate"
        self.target: Pose = target

    def ground(self) -> Motion:
        """
        Default resolver for moving the robot, this resolver simply creates the motion designator from the input of the
        designator description.

        :return: A resolved and performable motion designator
        """
        return self.Motion(self.cmd, self.target)


class PickUpMotion(MotionDesignatorDescription):
    """
    Lets the robot pick up a specific object
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_desig: ObjectDesignatorDescription.Object
        """
        Object designator describing the object to be picked up
        """
        arm: str
        """
        Arm that should be used for picking up the object
        """
        grasp: str
        """
        From which direction the object should be grasped, e.g. 'left', 'front', etc.
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.pick_up().execute(self)

    def __init__(self, object_desig: ObjectDesignatorDescription.Object, grasp: str = None, arm: str = None,
                 resolver: Callable = None):
        """
        Motion designator implementation of the robot picking up an object. The robot will move its arm to the position
        of the object and attach the object to the gripper of the robot.

        :param object_desig: Object designator of the object to be picked up
        :param grasp: From which direction the object should be picked up
        :param arm: Which arm should be used for picking up
        :param resolver: Alternative resolver that produces a resolved and performable motion deisgnator
        """
        super().__init__(resolver)
        self.cmd: str = 'pick-up'
        self.object_desig: ObjectDesignatorDescription.Object = object_desig
        self.arm: Optional[str] = arm
        self.grasp: Optional[str] = grasp

    def ground(self):
        """
        Default resolver for picking up. Checks if all parameter are present and will fill missing parameter. Optional
        parameter ``arm`` and ``grasp`` will default to ``'left'``.

        :return: A resolved motion designator that can be performed
        """
        arm = "left" if not self.arm else self.arm
        grasp = "left" if not self.grasp else self.grasp
        return self.Motion(self.cmd, self.object_desig, arm, grasp)


class PlaceMotion(MotionDesignatorDescription):
    """
    Lets the robot place an object that was picked up
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object: ObjectDesignatorDescription.Object
        """
        Object designator of the object to be placed
        """
        target: Pose
        """
        Pose at which the object should be placed
        """
        arm: str
        """
        Arm that is currently holding the object
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.place().execute(self)

    def __init__(self, object_desig: ObjectDesignatorDescription.Object, target: Pose,
                 arm: Optional[str] = None, resolver: Optional[Callable] = None):
        """
        Places the object in object_desig at the position in target. If an arm is given then the arm is used, otherwise
        arm defaults to ``'left'``

        :param object_desig: Object designator describing the object to be placed
        :param target: The target pose on which to place the object
        :param arm: An arm to use for placing
        :param resolver: An alternative resolver that resolves the list of parameters to a resolved motion designator.
        """
        super().__init__(resolver)
        self.cmd: str = 'place'
        self.object_desig: ObjectDesignatorDescription.Object = object_desig
        self.target: Pose = target
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for placing an object which returns a resolved motion designator for the input. If no arm is
        given then the arm parameter will default to ``'left'``.

        :return: A resolved performable motion designator
        """
        arm = "left" if not self.arm else self.arm
        return self.Motion(self.cmd, self.object_desig, self.target, arm)


class MoveTCPMotion(MotionDesignatorDescription):
    """
    Moves the Tool center point (TCP) of the robot
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        target: Pose
        """
        Target pose to which the TCP should be moved
        """
        arm: str
        """
        Arm with the TCP that should be moved to the target
        """
        allow_gripper_collision: bool
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
            motion.pose_id = pose.id

            session.add(motion)
            session.commit()

            return motion

    def __init__(self, target: Pose, arm: Optional[str] = None,
                 resolver: Optional[Callable] = None, allow_gripper_collision: Optional[bool] = None):
        """
        Moves the TCP of the given arm to the given target pose.

        :param target: Target pose for the TCP
        :param arm: Arm that should be moved
        :param resolver: Alternative resolver which returns a resolved motion designator
        :param allow_gripper_collision: If the gripper should be allowed to collide with something, only used on the real robot
        """
        super().__init__(resolver)
        self.cmd: str = 'move-tcp'
        self.target: Pose = target
        self.arm: Optional[str] = arm
        self.allow_gripper_collision = allow_gripper_collision

    def ground(self) -> Motion:
        """
        Default resolver that returns a resolved motion designator, arm defaults to ``'left'`` if no arm is given.

        :return: A resolved motion designator
        """
        arm = "left" if not self.arm else self.arm
        return self.Motion(self.cmd, self.target, arm, self.allow_gripper_collision)


class LookingMotion(MotionDesignatorDescription):
    """
    Lets the robot look at a point
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
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
            motion.pose_id = pose.id

            session.add(motion)
            session.commit()

            return motion

    def __init__(self, target: Optional[Pose] = None, object: Optional[ObjectDesignatorDescription.Object] = None,
                 resolver: Optional[Callable] = None):
        """
        Moves the head of the robot such that the camera points towards the given location. If ``target`` and ``object``
        are given ``target`` will be preferred.

        :param target: Position and orientation of the target
        :param object: An Object in the BulletWorld
        :param resolver: Alternative resolver that returns a resolved motion designator for parameter
        """
        super().__init__(resolver)
        self.cmd: str = 'looking'
        self.target: Optional[Pose] = target
        self.object: Object = object.bullet_world_object if object else object

    def ground(self) -> Motion:
        """
        Default resolver for looking, chooses which pose to take if ``target`` and ``object`` are given. If both are given
        ``target`` will be preferred.

        :return: A resolved motion designator
        """
        if not self.target and self.object:
            self.target = self.object.get_pose()
        return self.Motion(self.cmd, self.target)


class MoveGripperMotion(MotionDesignatorDescription):
    """
    Opens or closes the gripper
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        motion: str
        """
        Motion that should be performed, either 'open' or 'close'
        """
        gripper: str
        """
        Name of the gripper that should be moved
        """
        allow_gripper_collision: bool
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
            session.commit()
            return motion

    def __init__(self, motion: str, gripper: str, resolver: Optional[Callable] = None,
                 allow_gripper_collision: Optional[bool] = None):
        """
        Moves the gripper into a given position.

        :param motion: Which motion to perform
        :param gripper: Name of the gripper that should be moved
        :param resolver: An alternative resolver that resolves the parameter to a motion designator
        """
        super().__init__(resolver)
        self.cmd: str = 'move-gripper'
        self.motion: str = motion
        self.gripper: str = gripper
        self.allow_gripper_collision = allow_gripper_collision

    def ground(self) -> Motion:
        """
        Default resolver for moving the gripper, simply returns a resolved motion designator

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.motion, self.gripper, self.allow_gripper_collision)


class DetectingMotion(MotionDesignatorDescription):
    """
    Tries to detect an object in the FOV of the robot
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_type: str
        """
        Type of the object that should be detected
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            bullet_world_object = pm_manager.detecting().execute(self)
            if not bullet_world_object:
                raise PerceptionObjectNotFound(
                    f"Could not find an object with the type {self.object_type} in the FOV of the robot")
            if ProcessModuleManager.execution_type == "real":
                return RealObject.Object(bullet_world_object.name, bullet_world_object.type,
                                                      bullet_world_object, bullet_world_object.get_pose())

            return ObjectDesignatorDescription.Object(bullet_world_object.name, bullet_world_object.type,
                                                      bullet_world_object)

        def to_sql(self) -> ORMDetectingMotion:
            return ORMDetectingMotion(self.object_type)

        def insert(self, session: Session, *args, **kwargs) -> ORMDetectingMotion:
            motion = super().insert(session)
            session.add(motion)
            session.commit()
            return motion

    def __init__(self, object_type: str, resolver: Optional[Callable] = None):
        """
        Checks for every object in the FOV of the robot if it fits the given object type. If the types match an object
        designator describing the object will be returned.

        :param object_type: Type of the object which should be detected
        :param resolver: An alternative resolver which returns a resolved motion designator
        """
        super().__init__(resolver)
        self.cmd: str = 'detecting'
        self.object_type: str = object_type

    def ground(self) -> Motion:
        """
        Default resolver for detecting, simply returns a resolver motion designator without checking.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.object_type)


class MoveArmJointsMotion(MotionDesignatorDescription):
    """
    Moves the joints of each arm into the given position
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        left_arm_poses: Dict[str, float]
        """
        Target positions for the left arm joints
        """
        right_arm_poses: Dict[str, float]
        """
        Target positions for the right arm joints
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.move_arm_joints().execute(self)

    def __init__(self, left_arm_config: Optional[str] = None, right_arm_config: Optional[str] = None,
                 left_arm_poses: Optional[dict] = None, right_arm_poses: Optional[dict] = None,
                 resolver: Optional[Callable] = None):
        """
        Moves the arm joints, target positions can be either be pre-defined configurations (like 'park') or a dictionary
        with joint names as keys and joint positions as values. If a configuration and a dictionary are given the
        dictionary will be preferred.

        :param left_arm_config: Target configuration for the left arm
        :param right_arm_config: Target configuration for the right arm
        :param left_arm_poses: Target Dict for the left arm
        :param right_arm_poses: Target Dict for the right arm
        :param resolver: An alternative resolver that returns a resolved motion designator for the given parameters.
        """
        super().__init__(resolver)
        self.cmd = 'move-arm-joints'
        self.left_arm_config: str = left_arm_config
        self.right_arm_config: str = right_arm_config
        self.left_arm_poses: Dict[str, float] = left_arm_poses
        self.right_arm_poses: Dict[str, float] = right_arm_poses

    def ground(self) -> Motion:
        """
        Default resolver for moving the arms, returns a resolved motion designator containing the target positions for
        the left and right arm joints.

        :return: A resolved and performable motion designator
        """
        left_poses = None
        right_poses = None

        if self.left_arm_poses:
            left_poses = self.left_arm_poses
        elif self.left_arm_config:
            left_poses = robot_description.get_static_joint_chain("left", self.left_arm_config)

        if self.right_arm_poses:
            right_poses = self.right_arm_poses
        elif self.right_arm_config:
            right_poses = robot_description.get_static_joint_chain("right", self.right_arm_config)
        return self.Motion(self.cmd, left_poses, right_poses)


class WorldStateDetectingMotion(MotionDesignatorDescription):
    """
    Detects an object based on the world state.
    """
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_type: str
        """
        Object type that should be detected
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.world_state_detecting().execute(self)

    def __init__(self, object_type: str, resolver: Optional[Callable] = None):
        """
        Tries to find an object using the belief state (BulletWorld), if there is an object in the belief state matching
        the given object type an object designator will be returned.

        :param object_type: The object type which should be detected
        :param resolver: An alternative resolver that returns a resolved motion designator for the input parameter
        """
        super().__init__(resolver)
        self.cmd: str = 'world-state-detecting'
        self.object_type: str = object_type

    def ground(self) -> Motion:
        """
        Default resolver for world state detecting which simply returns a resolved motion designator for the input
        parameter.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.object_type)


class MoveJointsMotion(MotionDesignatorDescription):
    """
    Moves any joint on the robot
    """
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
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

    def __init__(self, names: List[str], positions: List[float], resolver: Optional[Callable] = None):
        """
        Moves the joints given by the list of names to the positions given by the list of positions. The index of a
        joint name should correspond to the index of the target position.

        :param names: List of joint names that should be moved
        :param positions: List of joint positions that the joints should be moved in
        :param resolver: An alternative resolver that resolves the input parameters to a performable motion designator.
        """
        super().__init__(resolver)
        self.cmd: str = "move-joints"
        self.names: List[str] = names
        self.positions: List[float] = positions

    def ground(self) -> Motion:
        """
        Default resolver for move joints, checks if the length of both list match and checks if the target positions are
        within the joint limits as stated in the URDF.

        :return: A resolved motion designator
        """
        if len(self.names) != len(self.positions):
            raise DesignatorError("[Motion Designator][Move Joints] The length of names and positions does not match")
        for i in range(len(self.names)):
            lower, upper = BulletWorld.robot.get_joint_limits(self.names[i])
            if self.positions[i] < lower or self.positions[i] > upper:
                raise DesignatorError(
                    f"[Motion Designator][Move Joints] The given configuration for the Joint {self.names[i]} violates its limits")
        return self.Motion(self.cmd, self.names, self.positions)


class OpeningMotion(MotionDesignatorDescription):
    """
    Designator for opening container
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_part: ObjectPart.Object
        """
        Object designator for the drawer handle
        """
        arm: str
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
            motion.object_id = op.id

            session.add(motion)
            session.commit()

            return motion

    def __init__(self, object_part: ObjectPart.Object, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot open a container specified by the given parameter. This motion designator assumes that the handle
        is already grasped.

        :param object_part: Object designator describing the handle of the drawer
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'open'
        self.objet_part = object_part
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.objet_part, self.arm)


class ClosingMotion(MotionDesignatorDescription):
    """
    Designator for closing a container
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_part: ObjectPart.Object
        """
        Object designator for the drawer handle
        """
        arm: str
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
            motion.object_id = op.id

            session.add(motion)
            session.commit()

            return motion

    def __init__(self, object_part: ObjectPart.Object, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot close a container specified by the given parameter. This assumes that the handle is already grasped

        :param object_part: Object designator describing the handle of the drawer
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'close'
        self.objet_part = object_part
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.objet_part, self.arm)
