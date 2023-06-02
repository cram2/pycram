import dataclasses
import itertools
from typing import List, Optional, Any, Tuple

import sqlalchemy.orm

from .motion_designator import *
from .object_designator import ObjectDesignatorDescription
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     Action as ORMAction)
from ..orm.base import Quaternion, Position, Base, RobotState
from ..robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from ..task import with_tree
from ..enums import Arms
from ..plan_failures import *
from ..bullet_world import BulletWorld


class ActionDesignatorDescription(DesignatorDescription):
    """
    Abstract class for action designator descriptions.
    Descriptions hold possible parameter ranges for action designators.
    """

    @dataclasses.dataclass
    class Action:
        """
        The performable designator with a single element for each list of possible parameter.
        """
        robot_position: Tuple[List[float], List[float]] = dataclasses.field(init=False)
        robot_torso_height: float = dataclasses.field(init=False)
        robot_type: str = dataclasses.field(init=False)
        """
        The position of the robot at the start of the action.
        """

        def __post_init__(self):
            self.robot_position = BulletWorld.robot.get_position_and_orientation()
            self.robot_torso_height = BulletWorld.robot.get_joint_state(robot_description.i.torso_joint)
            self.robot_type = BulletWorld.robot.type

        @with_tree
        def perform(self) -> Any:
            """
            Executes the action with the single parameters from the description.
            """
            raise NotImplementedError()

        def to_sql(self) -> ORMAction:
            """
            Create an ORM object that corresponds to this description.

            :return: The created ORM object.
            """
            raise NotImplementedError(f"{type(self)} has no implementation of to_sql. Feel free to implement it.")

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMAction:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :param args: Possible extra arguments
            :param kwargs: Possible extra keyword arguments
            :return: The completely instanced ORM object
            """

            # create position and orientation
            position = Position(*self.robot_position[0])
            orientation = Quaternion(*self.robot_position[1])
            session.add_all([position, orientation])
            session.commit()

            # create robot-state object
            robot_state = RobotState()
            robot_state.position = position.id
            robot_state.orientation = orientation.id
            robot_state.torso_height = self.robot_torso_height
            robot_state.type = self.robot_type
            session.add(robot_state)
            session.commit()

            # create action
            action = self.to_sql()
            action.robot_state = robot_state.id

            return action

    def __init__(self, resolver=None):
        super(ActionDesignatorDescription, self).__init__(resolver)

    def ground(self) -> Action:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


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
            MoveJointsMotion([robot_description.i.torso_joint], [self.position]).resolve().perform()

        def to_sql(self):
            return ORMMoveTorsoAction(self.position)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
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

        def to_sql(self) -> Base:
            return ORMSetGripperAction(self.gripper, self.motion)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
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
        super(SetGripperAction, self).__init__(resolver)
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
        super(GripAction, self).__init__(resolver)
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
        super(ParkArmsAction, self).__init__(resolver)
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
        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The grasp that should be used. For example, 'left' or 'right'
        """

        @with_tree
        def perform(self) -> None:
            self.object_at_execution = self.object_designator.data_copy()
            PickUpMotion(object_desig=self.object_designator, arm=self.arm, grasp=self.grasp).resolve().perform()

        def to_sql(self) -> ORMPickUpAction:
            return ORMPickUpAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            action = super().insert(session)
            # try to create the object designator
            if self.object_at_execution:
                od = self.object_at_execution.insert(session, )
                action.object = od.id
            else:
                action.object = None

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super(PickUpAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(), self.arms[0], self.grasps[0])


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
        target_location: Tuple[List[float], List[float]]
        """
        Pose in the world at which the object should be placed
        """

        @with_tree
        def perform(self) -> None:
            PlaceMotion(object_desig=self.object_designator, arm=self.arm, target=self.target_location).resolve(). \
                perform()

        def to_sql(self) -> ORMPlaceAction:
            return ORMPlaceAction(self.arm)

        def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
            action = super().insert(session)

            if self.object_designator:
                od = self.object_designator.insert(session, )
                action.object = od.id
            else:
                action.object = None

            if self.target_location:
                position = Position(*self.target_location[0])
                orientation = Quaternion(*self.target_location[1])
                session.add(position)
                session.add(orientation)
                session.commit()
                action.position = position.id
                action.orientation = orientation.id
            else:
                action.position = None
                action.orientation = None

            session.add(action)
            session.commit()
            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 target_locations: List[Tuple[List[float], List[float]]],
                 arms: List[str], resolver=None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        """
        super(PlaceAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.target_locations: List[Tuple[List[float], List[float]]] = target_locations
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(), self.arms[0],
                           self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Tuple[List[float], List[float]]
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

        def to_sql(self) -> ORMNavigateAction:
            return ORMNavigateAction()

        def insert(self, session, *args, **kwargs) -> ORMNavigateAction:

            # initialize position and orientation
            position = Position(*self.target_location[0])
            orientation = Quaternion(*self.target_location[1])

            # add those to the database and get the primary keys
            session.add(position)
            session.add(orientation)
            session.commit()

            # create the navigate action orm object
            action = super().insert(session)

            # set foreign keys
            action.position = position.id
            action.orientation = orientation.id

            # add it to the db
            session.add(action)
            session.commit()

            return action

    def __init__(self, target_locations: List[Tuple[List[float], List[float]]], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super(NavigateAction, self).__init__(resolver)
        self.target_locations: List[Tuple[List[float], List[float]]] = target_locations

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
        target_location: Tuple[List[float], List[float]]
        """
        Target Location to which the object should be transported
        """

        @with_tree
        def perform(self) -> None:
            # TODO jonas has to look over this
            ParkArmsAction.Action(Arms.BOTH).perform()
            NavigateAction.Action(self.object_designator.pose).perform()
            # TODO write from_object method for descriptions
            PickUpAction(self.object_designator, [self.arm], ["left", "right"]).ground().perform()
            NavigateAction.Action(self.target_location).perform()
            PlaceAction.Action(self.object_designator, self.arm, self.target_location)
            ParkArmsAction.Action(Arms.BOTH).perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 target_locations: List[Tuple[List[float], List[float]]], resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator describing the object that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super(TransportAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Tuple[List[float], List[float]]] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(),
                           self.arms[0],
                           self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target: List[float]
        """
        Position at which the robot should look, given as 6D pose
        """

        @with_tree
        def perform(self) -> None:
            LookingMotion(target=self.target).resolve().perform()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, targets: List[List[float]], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super(LookAtAction, self).__init__(resolver)
        self.targets: List[List[float]] = targets

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

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, resolver=None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative resolver
        """
        super(DetectAction, self).__init__(resolver)
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

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be opened
        """
        arm: str
        """
        Arm that should be used for opening the container
        """
        distance: float
        """
        Distance by which the container should be opened
        """

        @with_tree
        def perform(self) -> Any:
            object_type = self.object_designator.type
            if object_type in ["container", "drawer"]:
                motion_type = "opening-prismatic"
            elif object_type in ["fridge"]:
                motion_type = "opening-rotational"
            else:
                raise NotImplementedError()
            joint, handle = get_container_joint_and_handle(self.object_designator)
            arm = "left" if self.arm == Arms.LEFT else "right"
            environment = self.object_designator.prop_value('part-of')

            ProcessModule.perform(MotionDesignator(
                [('type', motion_type), ('joint', joint),
                 ('handle', handle), ('arm', arm), ('distance', self.distance),
                 ('part-of', environment)]))

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 distances: List[float], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the object that should be opened
        :param arms: A list of possible arms that should be used
        :param distances: A list of possible distances by which the container should be opened
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super(OpenAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.distances: List[float] = distances

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0], self.distances[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be closed
        """
        arm: str
        """
        Arm that should be used for closing
        """

        def perform(self) -> Any:
            object_type = self.object_designator.prop_value('type')
            if object_type in ["container", "drawer"]:
                motion_type = "closing-prismatic"
            elif object_type in ["fridge"]:
                motion_type = "closing-rotational"
            else:
                raise NotImplementedError()
            joint, handle = get_container_joint_and_handle(self.object_designator)
            arm = "left" if self.arm == Arms.LEFT else "right"
            environment = self.object_designator.prop_value('part-of')
            ProcessModule.perform(MotionDesignator(
                [('type', motion_type), ('joint', joint),
                 ('handle', handle), ('arm', arm), ('part-of', environment)]))

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str],
                 resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the object that should be closed
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super(CloseAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


def get_container_joint_and_handle(container_designator: Any):
    """
    Gets names of container joint and handle.
    TODO move this where it belongs

    :param container_designator: The object designator to get the names from.
    :return: (joint_name, handle_name)
    """
    name = container_designator.prop_value('name')
    if name == "iai_fridge":
        return "iai_fridge_door_joint", "iai_fridge_door_handle"
    elif name == "sink_area_left_upper_drawer":
        return "sink_area_left_upper_drawer_main_joint", "sink_area_left_upper_drawer_handle"
    elif name == "sink_area_left_middle_drawer":
        return "sink_area_left_middle_drawer_main_joint", "sink_area_left_middle_drawer_handle"
    else:
        raise NotImplementedError()
