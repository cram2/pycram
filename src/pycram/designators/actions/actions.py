import abc
import inspect

import numpy as np
from tf import transformations
from typing_extensions import Union, Type
from pycram.designator import ActionDesignatorDescription
from pycram.designators.motion_designator import *
from pycram.enums import Arms, Grasp
from pycram.task import with_tree
from dataclasses import dataclass, field
from ..location_designator import CostmapLocation
from ..object_designator import BelieveObject
from ...bullet_world import BulletWorld
from ...helper import multiply_quaternions
from ...local_transformer import LocalTransformer
from ...orm.base import Pose as ORMPose
from ...orm.object_designator import Object as ORMObject, ObjectPart as ORMObjectPart
from ...orm.action_designator import Action as ORMAction
from ...plan_failures import ObjectUnfetchable, ReachabilityFailure
from ...robot_descriptions import robot_description
from ...orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                      PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                      MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                      LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                      TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                      CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                      FaceAtAction as ORMFaceAtAction)


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable actions."""
    orm_class: Type[ORMAction] = field(init=False, default=None)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def perform(self) -> None:
        """
        Perform the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
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

        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
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
    def perform(self) -> None:
        MoveJointsMotion([robot_description.torso_joint], [self.position]).perform()


@dataclass
class SetGripperActionPerformable(ActionAbstract):
    """
    Set the gripper state of the robot.
    """

    gripper: str
    """
    The gripper that should be set 
    """
    motion: str
    """
    The motion that should be set on the gripper
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMSetGripperAction)

    @with_tree
    def perform(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: str

    object_designator: ObjectDesignatorDescription.Object

    def perform(self) -> None:
        raise NotImplementedError


@dataclass
class GripActionPerformable(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """

    gripper: str
    object_designator: ObjectDesignatorDescription.Object
    effort: float

    @with_tree
    def perform(self) -> None:
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
    def perform(self) -> None:
        # create the keyword arguments
        kwargs = dict()
        left_poses = None
        right_poses = None

        # add park left arm if wanted
        if self.arm in [Arms.LEFT, Arms.BOTH]:
            kwargs["left_arm_config"] = "park"
            left_poses = robot_description.get_static_joint_chain("left", kwargs["left_arm_config"])

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            right_poses = robot_description.get_static_joint_chain("right", kwargs["right_arm_config"])

        MoveArmJointsMotion(left_poses, right_poses).perform()


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be picked up
    """

    arm: str
    """
    The arm that should be used for pick up
    """

    grasp: str
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    @with_tree
    def perform(self) -> None:
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.frozen_copy()
        robot = BulletWorld.robot
        # Retrieve object and robot from designators
        object = self.object_designator.bullet_world_object
        # Get grasp orientation and target pose
        grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)
        # oTm = Object Pose in Frame map
        oTm = object.get_pose()
        # Transform the object pose to the object frame, basically the origin of the object frame
        mTo = object.local_transformer.transform_to_object_frame(oTm, object)
        # Adjust the pose according to the special knowledge of the object designator
        adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
        # Transform the adjusted pose to the map frame
        adjusted_oTm = object.local_transformer.transform_pose(adjusted_pose, "map")
        # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
        ori = multiply_quaternions([adjusted_oTm.orientation.x, adjusted_oTm.orientation.y,
                                    adjusted_oTm.orientation.z, adjusted_oTm.orientation.w],
                                   grasp)

        # Set the orientation of the object pose by grasp in MAP
        adjusted_oTm.orientation.x = ori[0]
        adjusted_oTm.orientation.y = ori[1]
        adjusted_oTm.orientation.z = ori[2]
        adjusted_oTm.orientation.w = ori[3]

        # prepose depending on the gripper (its annoying we have to put pr2_1 here tbh
        # gripper_frame = "pr2_1/l_gripper_tool_frame" if self.arm == "left" else "pr2_1/r_gripper_tool_frame"
        gripper_frame = robot.get_link_tf_frame(robot_description.get_tool_frame(self.arm))
        # First rotate the gripper, so the further calculations makes sense
        tmp_for_rotate_pose = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
        tmp_for_rotate_pose.pose.position.x = 0
        tmp_for_rotate_pose.pose.position.y = 0
        tmp_for_rotate_pose.pose.position.z = -0.1
        gripper_rotate_pose = object.local_transformer.transform_pose(tmp_for_rotate_pose, "map")

        #Perform Gripper Rotate
        # BulletWorld.current_bullet_world.add_vis_axis(gripper_rotate_pose)
        # MoveTCPMotion(gripper_rotate_pose, self.arm).resolve().perform()

        oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
        oTg.pose.position.x -= 0.1 # in x since this is how the gripper is oriented
        prepose = object.local_transformer.transform_pose(oTg, "map")

        # Perform the motion with the prepose and open gripper
        BulletWorld.current_bullet_world.add_vis_axis(prepose)
        MoveTCPMotion(prepose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(motion="open", gripper=self.arm).perform()

        # Perform the motion with the adjusted pose -> actual grasp and close gripper
        BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()
        adjusted_oTm.pose.position.z += 0.03
        MoveGripperMotion(motion="close", gripper=self.arm).perform()
        tool_frame = robot_description.get_tool_frame(self.arm)
        robot.attach(object, tool_frame)

        # Lift object
        BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()

        # Remove the vis axis from the world
        BulletWorld.current_bullet_world.remove_vis_axis()


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be place
    """
    arm: str
    """
    Arm that is currently holding the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    @with_tree
    def perform(self) -> None:
        object_pose = self.object_designator.bullet_world_object.get_pose()
        local_tf = LocalTransformer()

        # Transformations such that the target position is the position of the object and not the tcp
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                BulletWorld.robot.get_link_tf_frame(
                                                    robot_description.get_tool_frame(self.arm)))
        target_diff = self.target_location.to_transform("target").inverse_times(
            tcp_to_object.to_transform("object")).to_pose()

        MoveTCPMotion(target_diff, self.arm).perform()
        MoveGripperMotion("open", self.arm).perform()
        BulletWorld.robot.detach(self.object_designator.bullet_world_object)
        retract_pose = local_tf.transform_pose(target_diff, BulletWorld.robot.get_link_tf_frame(
            robot_description.get_tool_frame(self.arm)))
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
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMNavigateAction)

    @with_tree
    def perform(self) -> None:
        MoveMotion(self.target_location).perform()


@dataclass
class TransportActionPerformable(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be transported.
    """
    arm: str
    """
    Arm that should be used
    """
    target_location: Pose
    """
    Target Location to which the object should be transported
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def perform(self) -> None:
        robot_desig = BelieveObject(names=[robot_description.name])
        ParkArmsActionPerformable(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig.resolve(),
                                     reachable_arm=self.arm)
        # Tries to find a pick-up posotion for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm in pose.reachable_arms:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionPerformable(pickup_pose.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, "front").perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve(),
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
    def perform(self) -> None:
        LookingMotion(target=self.target).perform()


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator loosely describing the object, e.g. only type. 
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    @with_tree
    def perform(self) -> None:
        return DetectingMotion(object_type=self.object_designator.type).perform()


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be opened
    """
    arm: str
    """
    Arm that should be used for opening the container
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be closed
    """
    arm: str
    """
    Arm that should be used for closing
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()


@dataclass
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """

    arm: str
    """
    The arm that should be used to grasp
    """
    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object Designator for the object that should be grasped
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)

    @with_tree
    def perform(self) -> None:
        if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig.part_pose
        else:
            object_pose = self.object_desig.bullet_world_object.get_pose()
        lt = LocalTransformer()
        gripper_name = robot_description.get_tool_frame(self.arm)

        object_pose_in_gripper = lt.transform_pose(object_pose,
                                                   BulletWorld.robot.get_link_tf_frame(gripper_name))

        pre_grasp = object_pose_in_gripper.copy()
        pre_grasp.pose.position.x -= 0.1

        MoveTCPMotion(pre_grasp, self.arm).perform()
        MoveGripperMotion("open", self.arm).perform()

        MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion("close", self.arm, allow_gripper_collision=True).perform()


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
    def perform(self) -> None:
        # get the robot position
        robot_position = BulletWorld.robot.pose

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

    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()