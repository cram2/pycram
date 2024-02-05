import abc
from typing_extensions import Union
from pycram.designator import ActionDesignatorDescription
from pycram.designators.motion_designator import *
from pycram.enums import Arms
from pycram.task import with_tree
from dataclasses import dataclass, field
from ..location_designator import CostmapLocation
from ..object_designator import BelieveObject
from ...bullet_world import BulletWorld
from ...helper import multiply_quaternions
from ...local_transformer import LocalTransformer
from ...orm.base import Base
from ...plan_failures import ObjectUnfetchable, ReachabilityFailure
from ...robot_descriptions import robot_description
from ...orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                      PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                      MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                      LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                      TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                      CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action)


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable actions."""

    @abc.abstractmethod
    def perform(self) -> None:
        """
        Perform the action. Will be overwritten by each action.
        """
        pass

    @abc.abstractmethod
    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent. Will be overwritten by each action.
        """
        pass

    @abc.abstractmethod
    def insert(self, session: Session, **kwargs) -> Action:
        """
        Insert this action into the database.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM object
        """

        action = super().insert(session)
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

    @with_tree
    def perform(self) -> None:
        MoveJointsMotion([robot_description.torso_joint], [self.position]).perform()

    def to_sql(self) -> ORMMoveTorsoAction:
        return ORMMoveTorsoAction(self.position)

    def insert(self, session: Session, **kwargs) -> ORMMoveTorsoAction:
        action = super().insert(session)
        session.add(action)
        session.commit()
        return action


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

    @with_tree
    def perform(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()

    def to_sql(self) -> ORMSetGripperAction:
        return ORMSetGripperAction(self.gripper, self.motion)

    def insert(self, session: Session, *args, **kwargs) -> ORMSetGripperAction:
        action = super().insert(session)
        session.add(action)
        session.commit()
        return action


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

    def to_sql(self) -> ORMParkArmsAction:
        raise NotImplementedError

    def insert(self, session: Session, **kwargs) -> ORMParkArmsAction:
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

    def to_sql(self) -> Base:
        raise NotImplementedError()

    def insert(self, session: Session, *args, **kwargs) -> Base:
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

    def to_sql(self) -> ORMParkArmsAction:
        return ORMParkArmsAction(self.arm.name)

    def insert(self, session: Session, **kwargs) -> ORMParkArmsAction:
        action = super().insert(session)
        session.add(action)
        session.commit()
        return action


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
        MoveTCPMotion(prepose, self.arm).perform()
        MoveGripperMotion(motion="open", gripper=self.arm).perform()

        # Perform the motion with the adjusted pose -> actual grasp and close gripper
        BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm).perform()
        adjusted_oTm.pose.position.z += 0.03
        MoveGripperMotion(motion="close", gripper=self.arm).perform()
        tool_frame = robot_description.get_tool_frame(self.arm)
        robot.attach(object, tool_frame)

        # Lift object
        BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
        MoveTCPMotion(adjusted_oTm, self.arm).perform()

        # Remove the vis axis from the world
        BulletWorld.current_bullet_world.remove_vis_axis()

    def to_sql(self) -> ORMPickUpAction:
        return ORMPickUpAction(self.arm, self.grasp)

    def insert(self, session: Session, **kwargs) -> ORMPickUpAction:
        action = super().insert(session)

        od = self.object_at_execution.insert(session)
        action.object_id = od.id

        session.add(action)
        session.commit()

        return action


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

    def to_sql(self) -> ORMPlaceAction:
        return ORMPlaceAction(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMPlaceAction:
        action = super().insert(session)

        od = self.object_designator.insert(session)
        action.object_id = od.id

        pose = self.target_location.insert(session)
        action.pose_id = pose.id

        session.add(action)
        session.commit()

        return action


@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """

    @with_tree
    def perform(self) -> None:
        MoveMotion(self.target_location).perform()

    def to_sql(self) -> ORMNavigateAction:
        return ORMNavigateAction()

    def insert(self, session: Session, *args, **kwargs) -> ORMNavigateAction:
        action = super().insert(session)

        pose = self.target_location.insert(session)
        action.pose_id = pose.id

        session.add(action)
        session.commit()

        return action


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

    @with_tree
    def perform(self) -> None:
        robot_desig = BelieveObject(names=[robot_description.name])
        # ParkArmsAction.Action(Arms.BOTH).perform()
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
        # ParkArmsAction.Action(Arms.BOTH).perform()
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

    def to_sql(self) -> ORMTransportAction:
        return ORMTransportAction(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMTransportAction:
        action = super().insert(session)

        od = self.object_designator.insert(session)
        action.object_id = od.id

        pose = self.target_location.insert(session)
        action.pose_id = pose.id

        session.add(action)
        session.commit()

        return action


@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """

    @with_tree
    def perform(self) -> None:
        LookingMotion(target=self.target).perform()

    def to_sql(self) -> ORMLookAtAction:
        return ORMLookAtAction()

    def insert(self, session: Session, *args, **kwargs) -> ORMLookAtAction:
        action = super().insert(session)

        pose = self.target.insert(session)
        action.pose_id = pose.id

        session.add(action)
        session.commit()
        return action


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator loosely describing the object, e.g. only type. 
    """

    @with_tree
    def perform(self) -> None:
        return DetectingMotion(object_type=self.object_designator.type).perform()

    def to_sql(self) -> ORMDetectAction:
        return ORMDetectAction()

    def insert(self, session: Session, *args, **kwargs) -> ORMDetectAction:
        action = super().insert(session)

        od = self.object_designator.insert(session)
        action.object_id = od.id

        session.add(action)
        session.commit()

        return action


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

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()

    def to_sql(self) -> ORMOpenAction:
        return ORMOpenAction(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMOpenAction:
        action = super().insert(session)

        op = self.object_designator.insert(session)
        action.object_id = op.id

        session.add(action)
        session.commit()

        return action


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

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()

    def to_sql(self) -> ORMCloseAction:
        return ORMCloseAction(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMCloseAction:
        action = super().insert(session)

        op = self.object_designator.insert(session)
        action.object_id = op.id

        session.add(action)
        session.commit()

        return action


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

    def to_sql(self) -> ORMGraspingAction:
        return ORMGraspingAction(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMGraspingAction:
        action = super().insert(session)

        od = self.object_desig.insert(session)
        action.object_id = od.id

        session.add(action)
        session.commit()

        return action
