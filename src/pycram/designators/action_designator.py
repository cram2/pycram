import dataclasses
import itertools
import time
from typing import List, Optional, Any, Tuple, Union

import sqlalchemy.orm

from .location_designator import CostmapLocation
from .motion_designator import *
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..local_transformer import LocalTransformer
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     Action as ORMAction, LookAtAction as ORMLookAtAction,
                                     DetectAction as ORMDetectAction, TransportAction as ORMTransportAction,
                                     OpenAction as ORMOpenAction, CloseAction as ORMCloseAction,
                                     GraspingAction as ORMGraspingAction, CuttingAction as ORMCuttingAction)

from ..orm.base import Quaternion, Position, Base, RobotState, ProcessMetaData
from ..plan_failures import ObjectUnfetchable, ReachabilityFailure
from ..robot_descriptions import robot_description
from ..task import with_tree
from ..enums import Arms
from ..designator import ActionDesignatorDescription
from ..bullet_world import BulletWorld
from ..pose import Pose
from ..helper import multiply_quaternions, axis_angle_to_quaternion


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Performable Move Torso Action designator.
        """

        position: float
        """
        Target position of the torso joint
        """

        @with_tree
        def perform(self) -> None:
            MoveJointsMotion([robot_description.torso_joint], [self.position]).resolve().perform()

        def to_sql(self) -> ORMMoveTorsoAction:
            return ORMMoveTorsoAction(self.position)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMMoveTorsoAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, positions: List[float], resolver=None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional resolver that returns a performable designator for a designator description.
        """
        super().__init__(resolver)
        self.positions: List[float] = positions

    def ground(self) -> Action:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return self.Action(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield self.Action(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
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
            MoveGripperMotion(gripper=self.gripper, motion=self.motion).resolve().perform()

        def to_sql(self) -> ORMSetGripperAction:
            return ORMSetGripperAction(self.gripper, self.motion)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMSetGripperAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, grippers: List[str], motions: List[str], resolver=None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative resolver that returns a performable designator for a designator description
        """
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.motions: List[str] = motions

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return self.Action(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield self.Action(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object
        effort: float

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        arm: Arms
        """
        Entry from the enum for which arm should be parked
        """

        @with_tree
        def perform(self) -> None:
            # create the keyword arguments
            kwargs = dict()

            # add park left arm if wanted
            if self.arm in [Arms.LEFT, Arms.BOTH]:
                kwargs["left_arm_config"] = "park"

            # add park right arm if wanted
            if self.arm in [Arms.RIGHT, Arms.BOTH]:
                kwargs["right_arm_config"] = "park"
            MoveArmJointsMotion(**kwargs).resolve().perform()

        def to_sql(self) -> ORMParkArmsAction:
            return ORMParkArmsAction(self.arm.name)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMParkArmsAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, arms: List[Arms], resolver=None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[Arms] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return self.Action(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
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

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()
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

            gripper_frame = robot.get_link_tf_frame(robot_description.get_tool_frame(self.arm))
            tmp_for_rotate_pose = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
            tmp_for_rotate_pose.pose.position.x = 0
            tmp_for_rotate_pose.pose.position.y = 0
            tmp_for_rotate_pose.pose.position.z = -0.1

            oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
            oTg.pose.position.x -= 0.07  # in x since this is how the gripper is oriented
            prepose = object.local_transformer.transform_pose(oTg, "map")

            # Perform the motion with the prepose and open gripper
            BulletWorld.current_bullet_world.add_vis_axis(prepose)
            MoveTCPMotion(prepose, self.arm).resolve().perform()
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Perform the motion with the adjusted pose -> actual grasp and close gripper
            BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
            MoveTCPMotion(adjusted_oTm, self.arm).resolve().perform()
            adjusted_oTm.pose.position.z += 0.03
            MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()
            tool_frame = robot_description.get_tool_frame(self.arm)
            robot.attach(object, tool_frame)

            # Lift object
            BulletWorld.current_bullet_world.add_vis_axis(adjusted_oTm)
            MoveTCPMotion(adjusted_oTm, self.arm).resolve().perform()

            # Remove the vis axis from the world
            BulletWorld.current_bullet_world.remove_vis_axis()

        def to_sql(self) -> ORMPickUpAction:
            return ORMPickUpAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMPickUpAction:
            action = super().insert(session)

            od = self.object_at_execution.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
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

            MoveTCPMotion(target_diff, self.arm).resolve().perform()
            MoveGripperMotion("open", self.arm).resolve().perform()
            BulletWorld.robot.detach(self.object_designator.bullet_world_object)
            retract_pose = local_tf.transform_pose(target_diff, BulletWorld.robot.get_link_tf_frame(
                robot_description.get_tool_frame(self.arm)))
            retract_pose.position.x -= 0.07
            MoveTCPMotion(retract_pose, self.arm).resolve().perform()

        def to_sql(self) -> ORMPlaceAction:
            return ORMPlaceAction(self.arm)

        def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
            action = super().insert(session)

            od = self.object_designator.insert(session)
            action.object_id = od.id

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[str], resolver=None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.target_locations: List[Pose] = target_locations
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Pose
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

        def to_sql(self) -> ORMNavigateAction:
            return ORMNavigateAction()

        def insert(self, session, *args, **kwargs) -> ORMNavigateAction:
            action = super().insert(session)

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return self.Action(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
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
            ParkArmsAction.Action(Arms.BOTH).perform()
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

            NavigateAction([pickup_pose.pose]).resolve().perform()
            PickUpAction.Action(self.object_designator, self.arm, "front").perform()
            ParkArmsAction.Action(Arms.BOTH).perform()
            try:
                place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve(),
                                            reachable_arm=self.arm).resolve()
            except StopIteration:
                raise ReachabilityFailure(
                    f"No location found from where the robot can reach the target location: {self.target_location}")
            NavigateAction([place_loc.pose]).resolve().perform()
            PlaceAction.Action(self.object_designator, self.arm, self.target_location).perform()
            ParkArmsAction.Action(Arms.BOTH).perform()

        def to_sql(self) -> ORMTransportAction:
            return ORMTransportAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMTransportAction:
            action = super().insert(session)

            od = self.object_designator.insert(session)
            action.object_id = od.id

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str],
                 target_locations: List[Pose], resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        return self.Action(obj_desig,
                           self.arms[0],
                           self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target: Pose
        """
        Position at which the robot should look, given as 6D pose
        """

        @with_tree
        def perform(self) -> None:
            LookingMotion(target=self.target).resolve().perform()

        def to_sql(self) -> ORMLookAtAction:
            return ORMLookAtAction()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMLookAtAction:
            action = super().insert(session)

            pose = self.target.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()
            return action

    def __init__(self, targets: List[Pose], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super().__init__(resolver)
        self.targets: List[Pose] = targets

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return self.Action(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator loosely describing the object, e.g. only type. 
        """

        @with_tree
        def perform(self) -> Any:
            return DetectingMotion(object_type=self.object_designator.type).resolve().perform()

        def to_sql(self) -> ORMDetectAction:
            return ORMDetectAction()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMDetectAction:
            action = super().insert(session)

            od = self.object_designator.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription, resolver=None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve())


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be opened
        """
        arm: str
        """
        Arm that should be used for opening the container
        """

        @with_tree
        def perform(self) -> Any:
            GraspingAction.Action(self.arm, self.object_designator).perform()
            OpeningMotion(self.object_designator, self.arm).resolve().perform()

            MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMOpenAction:
            return ORMOpenAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMOpenAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be closed
        """
        arm: str
        """
        Arm that should be used for closing
        """

        @with_tree
        def perform(self) -> Any:
            GraspingAction.Action(self.arm, self.object_designator).perform()
            ClosingMotion(self.object_designator, self.arm).resolve().perform()

            MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMCloseAction:
            return ORMCloseAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMCloseAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str],
                 resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        arm: str
        """
        The arm that should be used to grasp
        """
        object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
        """
        Object Designator for the object that should be grasped
        """

        def perform(self) -> Any:
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

            MoveTCPMotion(pre_grasp, self.arm).resolve().perform()
            MoveGripperMotion("open", self.arm).resolve().perform()

            MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).resolve().perform()
            MoveGripperMotion("close", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMGraspingAction:
            return ORMGraspingAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMGraspingAction:
            action = super().insert(session)

            od = self.object_desig.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, arms: List[str], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative resolver to get a specified designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[str] = arms
        self.object_description: ObjectDesignatorDescription = object_description

    def ground(self) -> Action:
        """
        Default resolver that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return self.Action(self.arms[0], self.object_description.resolve())


class CuttingAction(ActionDesignatorDescription):
    """
    A designator for robotic cutting actions. This class facilitates the specification and execution of
    cutting tasks using a robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Represents a specific cutting action to be performed by the robot. This class encapsulates
        all necessary details for executing the cutting task, including the object to be cut, the cutting tool,
        the arm to use, and the cutting technique.
        """

        object_to_be_cut: ObjectDesignatorDescription.Object
        """
        object_to_be_cut (ObjectDesignatorDescription.Object): The object to be cut.
        """
        tool: ObjectDesignatorDescription.Object
        """
        tool (ObjectDesignatorDescription.Object): The tool used for cutting.
        """
        arm: str
        """
        arm (str): The robot arm designated for the cutting task.
        """
        technique: Optional[str] = None
        """
        technique (Optional[str]): The technique used for cutting (default is None).
        """
        slice_thickness: Optional[float] = None
        """
        slice_thickness (Optional[float]): The upper bound thickness of the slices (default is None).
        """
        object_to_be_cut_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object_to_be_cut at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """
        tool_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object_tool at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            """
            Executes the cutting action using the specified parameters. It involves determining the
            grasp orientation, computing slice positions, and orchestrating the necessary robot motions
            for executing the cutting task.
            """

            self.object_to_be_cut_at_execution = self.object_to_be_cut.data_copy()
            self.tool_at_execution = self.tool.data_copy()

            # Get grasp orientation and target pose
            grasp = robot_description.grasps.get_orientation_for_grasp("top")
            # Retrieve object and robot from designators
            object = self.object_to_be_cut.bullet_world_object
            obj_dim = object.get_object_dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            # from bread_dim calculate def a calculation that gets me the highest number from the first 2 entries
            # Given slice thickness is 3 cm or 0.03 meters
            slice_thickness = self.slice_thickness
            # Calculate slices and transform them to the map frame with orientation
            obj_length = dim[0]
            obj_width = dim[1]
            obj_height = dim[2]

            # Calculate the starting Y-coordinate offset (half the width minus half a slice thickness)
            if self.technique == 'halving':
                start_offset = 0
                num_slices = 1
            else:
                num_slices = int(obj_length // slice_thickness)
                start_offset = (-obj_length / 2 + slice_thickness / 2)

            # Calculate slice coordinates
            slice_coordinates = [start_offset + i * slice_thickness for i in range(num_slices)]

            # Transform slice coordinates to map frame with orientation
            slice_poses = []
            for x in slice_coordinates:
                tmp_pose = object_pose.copy()
                tmp_pose.pose.position.y -= 3 * obj_width
                tmp_pose.pose.position.x = x
                sTm = object.local_transformer.transform_pose(tmp_pose, "map")
                slice_poses.append(sTm)

            for slice_pose in slice_poses:
                # rotate the slice_pose by grasp
                ori = multiply_quaternions(
                    [slice_pose.orientation.x, slice_pose.orientation.y, slice_pose.orientation.z,
                     slice_pose.orientation.w], grasp)

                oriR = axis_angle_to_quaternion([0, 0, 1], 90)
                oriM = multiply_quaternions([oriR[0], oriR[1], oriR[2], oriR[3]], [ori[0], ori[1], ori[2], ori[3]])

                adjusted_slice_pose = slice_pose.copy()

                # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = oriM[0]
                adjusted_slice_pose.orientation.y = oriM[1]
                adjusted_slice_pose.orientation.z = oriM[2]
                adjusted_slice_pose.orientation.w = oriM[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += 2 * obj_height
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()
                # Perform the motion for cutting the object
                BulletWorld.current_bullet_world.add_vis_axis(adjusted_slice_pose)
                MoveTCPMotion(adjusted_slice_pose, self.arm).resolve().perform()
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

        def to_sql(self) -> ORMCuttingAction:
            """
            Convert the action to a corresponding SQL representation for storage.
            """
            return ORMCuttingAction(self.arm, self.technique, self.slice_thickness)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            """
            Insert the cutting action into the database session.
            """
            # insert related objects
            object_to_be_cut = self.object_to_be_cut_at_execution.insert(session)
            tool = self.tool_at_execution.insert(session)

            action = super().insert(session)
            action.object_to_be_cut_id = object_to_be_cut.id
            action.tool_id = tool.id

            # Additional logic for inserting cutting action data goes here
            session.add(action)
            session.commit()

            return action

    def __init__(self, object_to_be_cut: ObjectDesignatorDescription,
                 tool: ObjectDesignatorDescription, arms: List[str], technique: Optional[str] = None):
        """
        Initializes a CuttingAction with specified object and tool designators, arms, and an optional cutting technique.

        Args:
            object_to_be_cut (ObjectDesignatorDescription): Designator for the object to be cut.
            tool (ObjectDesignatorDescription): Designator for the cutting tool.
            arms (List[str]): List of possible arms to be used for the cutting action.
            technique (Optional[str]): Optional cutting technique to be used.
        """
        super(CuttingAction, self).__init__()
        self.object_to_be_cut: ObjectDesignatorDescription = object_to_be_cut
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[str] = arms
        self.technique: Optional[str] = technique

    def __iter__(self):
        """
        Iterator for generating all possible action combinations based on the provided object, tool, and arms.

        Yields:
        Action: A possible cutting action with a specific combination of object, tool, and arm.
        """
        for object_, tool_, arm in itertools.product(iter(self.object_to_be_cut),iter(self.tool), self.arms):
            yield self.Action(object_, tool_, arm)

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return next(iter(self))
