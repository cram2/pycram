# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
import itertools

import numpy as np
from sqlalchemy.orm import Session
from tf import transformations
from typing_extensions import List, Union, Callable, Optional, Type

from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveArmJointsMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.property import GraspableProperty, ReachableProperty, GripperIsFreeProperty, SpaceIsFreeProperty, \
    VisibleProperty
from ..knowledge.knowledge_engine import ReasoningInstance
from ..local_transformer import LocalTransformer
from ..failures import ObjectUnfetchable, ReachabilityFailure
from ..robot_description import RobotDescription
from ..tasktree import with_tree

from owlready2 import Thing

from ..datastructures.enums import Arms, Grasp, GripperState, MovementType
from ..designator import ActionDesignatorDescription
from ..datastructures.pose import Pose
from ..datastructures.world import World

from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                     TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                     CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                     FaceAtAction as ORMFaceAtAction)
from ..orm.base import Pose as ORMPose
from ..orm.object_designator import Object as ORMObject
from ..orm.action_designator import Action as ORMAction
from dataclasses import dataclass, field


# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable performables."""
    orm_class: Type[ORMAction] = field(init=False, default=None, repr=False)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def plan(self) -> None:
        """
        plan of the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # list of parameters that will be passed to the ORM class. If the name does not match the orm_class equivalent
        # or if it is a type that needs to be inserted into the session manually, it will not be added to the list
        parameters = [value for key, value in class_variables.items() if key in orm_class_variables
                      and not isinstance(value, (ObjectDesignatorDescription.Object, Pose))]

        return self.orm_class(*parameters)

    def insert(self, session: Session, **kwargs) -> Action:
        """
        Insert this action into the database.

        Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
        equivalent. In that case, the attributes need to be inserted into the session manually.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM action that was inserted into the database
        """

        action = super().insert(session)

        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # loop through all class parameters and insert them into the session unless they are already added by the ORM
        for key, value in class_variables.items():
            if key not in orm_class_variables:
                variable = value.insert(session)
                if isinstance(variable, ORMObject):
                    action.object = variable
                elif isinstance(variable, ORMPose):
                    action.pose = variable
        session.add(action)

        return action


@dataclass
class MoveTorsoActionPerformable(ActionAbstract):
    """
    Move the torso of the robot up and down.
    """

    position: float
    """
    Target position of the torso joint
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMMoveTorsoAction)

    @with_tree
    def plan(self) -> None:
        MoveJointsMotion([RobotDescription.current_robot_description.torso_joint], [self.position]).perform()


@dataclass
class SetGripperActionPerformable(ActionAbstract):
    """
    Set the gripper state of the robot.
    """

    gripper: Arms
    """
    The gripper that should be set 
    """
    motion: GripperState
    """
    The motion that should be set on the gripper
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMSetGripperAction)

    @with_tree
    def plan(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: Arms

    object_designator: ObjectDesignatorDescription.Object

    def plan(self) -> None:
        raise NotImplementedError


@dataclass
class GripActionPerformable(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """

    gripper: Arms
    object_designator: ObjectDesignatorDescription.Object
    effort: float

    @with_tree
    def plan(self) -> None:
        raise NotImplementedError()


@dataclass
class ParkArmsActionPerformable(ActionAbstract):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMParkArmsAction)

    @with_tree
    def plan(self) -> None:
        # create the keyword arguments
        kwargs = dict()
        left_poses = None
        right_poses = None

        # add park left arm if wanted
        if self.arm in [Arms.LEFT, Arms.BOTH]:
            kwargs["left_arm_config"] = "park"
            left_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_static_joint_states(
                kwargs["left_arm_config"])

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            right_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_static_joint_states(
                kwargs["right_arm_config"])

        MoveArmJointsMotion(left_poses, right_poses).perform()


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp: Grasp
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    prepose_distance: float = 0.07
    """
    The distance in meters the gripper should be at before picking up the object
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.frozen_copy()

    @with_tree
    def plan(self) -> None:
        robot = World.robot
        # Retrieve object and robot from designators
        object = self.object_designator.world_object
        # Get grasp orientation and target pose
        grasp = RobotDescription.current_robot_description.grasps[self.grasp]
        # oTm = Object Pose in Frame map
        oTm = object.get_pose()
        # Transform the object pose to the object frame, basically the origin of the object frame
        mTo = object.local_transformer.transform_to_object_frame(oTm, object)
        # Adjust the pose according to the special knowledge of the object designator_description
        adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
        # Transform the adjusted pose to the map frame
        adjusted_oTm = object.local_transformer.transform_pose(adjusted_pose, "map")
        # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper

        adjusted_oTm.multiply_quaternions(grasp)

        # prepose depending on the gripper (its annoying we have to put pr2_1 here tbh
        arm_chain = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        gripper_frame = robot.get_link_tf_frame(arm_chain.get_tool_frame())

        oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
        oTg.pose.position.x -= 0.1  # in x since this is how the gripper is oriented
        prepose = object.local_transformer.transform_pose(oTg, "map")

        # Perform the motion with the prepose and open gripper
        World.current_world.add_vis_axis(prepose)
        MoveTCPMotion(prepose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        # Perform the motion with the adjusted pose -> actual grasp and close gripper
        World.current_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()
        adjusted_oTm.pose.position.z += 0.03
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm).perform()
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        robot.attach(object, tool_frame)

        # Lift object
        World.current_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()

        # Remove the vis axis from the world
        World.current_world.remove_vis_axis()

    # TODO find a way to use object_at_execution instead of object_designator in the automatic orm mapping in ActionAbstract
    def to_sql(self) -> Action:
        return ORMPickUpAction(arm=self.arm, grasp=self.grasp)

    def insert(self, session: Session, **kwargs) -> Action:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.object_at_execution.insert(session)

        session.add(action)
        return action


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be place
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    @with_tree
    def plan(self) -> None:
        object_pose = self.object_designator.world_object.get_pose()
        local_tf = LocalTransformer()

        # Transformations such that the target position is the position of the object and not the tcp
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                World.robot.get_link_tf_frame(
                                                    RobotDescription.current_robot_description.get_arm_chain(
                                                        self.arm).get_tool_frame()))
        target_diff = self.target_location.to_transform("target").inverse_times(
            tcp_to_object.to_transform("object")).to_pose()

        MoveTCPMotion(target_diff, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        World.robot.detach(self.object_designator.world_object)
        retract_pose = local_tf.transform_pose(target_diff, World.robot.get_link_tf_frame(
            RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()))
        retract_pose.position.x -= 0.07
        MoveTCPMotion(retract_pose, self.arm).perform()


@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """

    keep_joint_states: bool = False
    """
    Keep the joint states of the robot the same during the navigation.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMNavigateAction)

    @with_tree
    def plan(self) -> None:
        MoveMotion(self.target_location, self.keep_joint_states).perform()


@dataclass
class TransportActionPerformable(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be transported.
    """
    target_location: Pose
    """
    Target Location to which the object should be transported
    """
    arm: Arms
    """
    Arm that should be used
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def plan(self) -> None:
        robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig_resolved,
                                     reachable_arm=self.arm)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm in pose.reachable_arms:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionPerformable(pickup_pose.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, Grasp.FRONT).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig_resolved,
                                        reachable_arm=self.arm).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateActionPerformable(place_loc.pose).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()


@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def plan(self) -> None:
        LookingMotion(target=self.target).perform()


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description loosely describing the object, e.g. only type. 
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    @with_tree
    def plan(self) -> None:
        return DetectingMotion(object_type=self.object_designator.obj_type).perform()


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be opened
    """
    arm: Arms
    """
    Arm that should be used for opening the container
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
    def plan(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be closed
    """
    arm: Arms
    """
    Arm that should be used for closing
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def plan(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object Designator for the object that should be grasped
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)

    @with_tree
    def plan(self) -> None:
        if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig.part_pose
        else:
            object_pose = self.object_desig.world_object.get_pose()
        lt = LocalTransformer()
        gripper_name = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()

        object_pose_in_gripper = lt.transform_pose(object_pose,
                                                   World.robot.get_link_tf_frame(gripper_name))

        pre_grasp = object_pose_in_gripper.copy()
        pre_grasp.pose.position.x -= 0.1

        MoveTCPMotion(pre_grasp, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()

        MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(GripperState.CLOSE, self.arm, allow_gripper_collision=True).perform()


@dataclass
class FaceAtPerformable(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """

    orm_class = ORMFaceAtAction

    @with_tree
    def plan(self) -> None:
        # get the robot position
        robot_position = World.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = Pose(robot_position.position_as_list(), orientation)

        # turn robot
        NavigateActionPerformable(new_robot_pose).perform()

        # look at target
        LookAtActionPerformable(self.pose).perform()


@dataclass
class MoveAndPickUpPerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    arm: Arms
    """
    The arm to use
    """

    grasp: Grasp
    """
    The grasp to use
    """

    # @with_tree
    def plan(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()


@dataclass
class MoveAndPlacePerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    target_location: Pose
    """
    The location to place the object.
    """

    arm: Arms
    """
    The arm to use
    """

    @with_tree
    def plan(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.target_location).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()




# ----------------------------------------------------------------------------
#               Action Designators Description
# ----------------------------------------------------------------------------


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """
    performable_class = MoveTorsoActionPerformable

    def __init__(self, positions: List[float]):
        """
        Create a designator_description description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        """
        super().__init__()
        self.positions: List[float] = positions

    def ground(self) -> MoveTorsoActionPerformable:
        """
        Creates a performable action designator_description with the first element from the list of possible torso heights.

        :return: A performable action designator_description
        """
        return MoveTorsoActionPerformable(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        for position in self.positions:
            yield MoveTorsoActionPerformable(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    performable_class = SetGripperActionPerformable

    def __init__(self, grippers: List[Arms], motions: List[GripperState]):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        """
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.motions: List[GripperState] = motions


    def ground(self) -> SetGripperActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first element in the grippers and motions list.

        :return: A performable designator_description
        """
        return SetGripperActionPerformable(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator_description with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield SetGripperActionPerformable(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    performable_class = ReleaseActionPerformable

    def __init__(self, object_designator_description: ObjectDesignatorDescription, grippers: List[Arms] = None):
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.object_designator_description = object_designator_description


    def ground(self) -> ReleaseActionPerformable:
        return ReleaseActionPerformable(self.grippers[0], self.object_designator_description.ground())

    def __iter__(self):
        ri = ReasoningInstance(self,
                                 PartialDesignator(ReleaseActionPerformable, self.grippers, self.object_designator_description))
        for desig in ri:
            yield desig


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    performable_class = GripActionPerformable

    def __init__(self, object_designator_description: ObjectDesignatorDescription, grippers: List[Arms] = None,
                 efforts: List[float] = None):
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts


    def ground(self) -> GripActionPerformable:
        return GripActionPerformable(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])

    def __iter__(self):
        ri = ReasoningInstance(self,
                                 PartialDesignator(GripActionPerformable, self.grippers, self.object_designator_description,
                                                     self.efforts))
        for desig in ri:
            yield desig


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    performable_class = ParkArmsActionPerformable

    def __init__(self, arms: List[Arms]):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        """
        super().__init__()
        self.arms: List[Arms] = arms


    def ground(self) -> ParkArmsActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first element of the list of possible arms

        :return: A performable designator_description
        """
        return ParkArmsActionPerformable(self.arms[0])

    def __iter__(self) -> ParkArmsActionPerformable:
        """
        Iterates over all possible solutions and returns a performable designator with the arm.

        :return: A performable designator_description
        """
        for arm in self.arms:
            yield ParkArmsActionPerformable(arm)


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    performable_class = PickUpActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms] = None, grasps: List[Grasp] = None):
        """
        Lets the robot pick up an object. The description needs an object designator_description describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator_description
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.grasps: List[Grasp] = grasps
        object_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                        ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        self.knowledge_condition = GraspableProperty(self.object_designator_description) & ReachableProperty(
            object_desig.pose)

    def __iter__(self) -> PickUpActionPerformable:
        ri = ReasoningInstance(self,
                               PartialDesignator(PickUpActionPerformable, self.object_designator_description, self.arms,
                                                 self.grasps))
        # Here is where the magic happens
        for desig in ri:
            yield desig


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    performable_class = PlaceActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[Arms] = None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        object_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                       ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        self.target_locations: List[Pose] = target_locations
        self.arms: List[Arms] = arms
        self.knowledge_condition = ReachableProperty(object_desig.pose)


    def ground(self) -> PlaceActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entries from the list of possible entries.

        :return: A performable designator_description
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.target_locations[0])

    def __iter__(self) -> PlaceActionPerformable:
        ri = ReasoningInstance(self,
                               PartialDesignator(PlaceActionPerformable, self.object_designator_description, self.arms,
                                                 self.target_locations))
        for desig in ri:
            yield desig


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    performable_class = NavigateActionPerformable

    def __init__(self, target_locations: List[Pose]):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        """
        super().__init__()
        self.target_locations: List[Pose] = target_locations
        if len(self.target_locations) == 1:
            self.knowledge_condition = SpaceIsFreeProperty(self.target_locations[0])
        else:
            root = SpaceIsFreeProperty(self.target_locations[0])
            for location in self.target_locations[1:]:
                root |= SpaceIsFreeProperty(location)
            self.knowledge_condition = root

    def ground(self) -> NavigateActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entry of possible target locations.

        :return: A performable designator_description
        """
        return NavigateActionPerformable(self.target_locations[0])

    def __iter__(self) -> NavigateActionPerformable:
        """
        Iterates over all possible target locations

        :return: A performable designator_description
        """
        for location in self.target_locations:
            yield NavigateActionPerformable(location)


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    performable_class = TransportActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[Arms] = None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator_description description or a specified Object designator_description that should be transported
        :param target_locations: A list of possible target locations for the object to be placed
        :param arms: A List of possible arms that could be used for transporting
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.target_locations: List[Pose] = target_locations


    def ground(self) -> TransportActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entries from the lists of possible parameter.

        :return: A performable designator_description
        """
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()

        return TransportActionPerformable(obj_desig, self.target_locations[0],  self.arms[0])

    def __iter__(self) -> TransportActionPerformable:
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()
        ri = ReasoningInstance(self,
                               PartialDesignator(TransportActionPerformable, obj_desig, self.target_locations,
                                                 self.arms))
        for desig in ri:
            yield desig


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    performable_class = LookAtActionPerformable

    def __init__(self, targets: List[Pose]):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        """
        super().__init__()
        self.targets: List[Pose] = targets


    def ground(self) -> LookAtActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entry in the list of possible targets

        :return: A performable designator_description
        """
        return LookAtActionPerformable(self.targets[0])

    def __iter__(self) -> LookAtActionPerformable:
        """
        Iterates over all possible target locations

        :return: A performable designator_description
        """
        for target in self.targets:
            yield LookAtActionPerformable(target)


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.
    """

    performable_class = DetectActionPerformable

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator_description describing the object
        """
        super().__init__()
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.knowledge_condition = VisibleProperty(self.object_designator_description)


    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object description.

        :return: A performable designator_description
        """
        return DetectActionPerformable(self.object_designator_description.resolve())

    def __iter__(self) -> DetectActionPerformable:
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        for desig in self.object_designator_description:
            yield DetectActionPerformable(desig)


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    performable_class = OpenActionPerformable

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms] = None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator_description describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        """
        super().__init__()
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.knowledge_condition = GripperIsFreeProperty(self.arms)

    def ground(self) -> OpenActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator_description
        """
        return OpenActionPerformable(self.object_designator_description.resolve(), self.arms[0])

    def __iter__(self) -> OpenActionPerformable:
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(OpenActionPerformable, self.object_designator_description, self.arms))
        for desig in ri:
            yield desig


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    performable_class = CloseActionPerformable

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms] = None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator_description description of the handle that should be used
        :param arms: A list of possible arms to use
        """
        super().__init__()
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.knowledge_condition = GripperIsFreeProperty(self.arms)


    def ground(self) -> CloseActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object designator_description and the first entry from
        the list of possible arms.

        :return: A performable designator_description
        """
        return CloseActionPerformable(self.object_designator_description.resolve(), self.arms[0])

    def __iter__(self) -> CloseActionPerformable:
        """
        Iterates over all possible solutions for this designator_description and returns a performable action designator.

        :yield: A performable fully parametrized Action designator
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(CloseActionPerformable, self.object_designator_description, self.arms))
        for desig in ri:
            yield desig


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    performable_class = GraspingActionPerformable

    def __init__(self, object_description: Union[ObjectDesignatorDescription, ObjectPart], arms: List[Arms] = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        """
        super().__init__()
        self.arms: List[Arms] = arms
        self.object_description: ObjectDesignatorDescription = object_description


    def ground(self) -> GraspingActionPerformable:
        """
        Default specialized_designators that takes the first element from the list of arms and the first solution for the object
        designator_description description ond returns it.

        :return: A performable action designator_description that contains specific arguments
        """
        return GraspingActionPerformable(self.arms[0], self.object_description.resolve())

    def __iter__(self) -> CloseActionPerformable:
        """
        Iterates over all possible solutions for this designator_description and returns a performable action
        designator.

        :yield: A fully parametrized Action designator
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(GraspingActionPerformable, self.object_description, self.arms))
        for desig in ri:
            yield desig

