# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
import itertools
import math

import numpy as np
from sqlalchemy.orm import Session
from tf import transformations
from typing_extensions import Any, List, Union, Callable, Optional, Type

from .location_designator import CostmapLocation, AccessingLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveArmJointsMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from .. import utils
from ..helper import calculate_object_faces, adjust_grasp_for_object_rotation, \
    calculate_grasp_offset
from ..local_transformer import LocalTransformer
from ..plan_failures import ObjectUnfetchable, ReachabilityFailure
from ..robot_description import RobotDescription
from ..ros.viz_marker_publisher import AxisMarkerPublisher
from ..tasktree import with_tree

from owlready2 import Thing

from ..datastructures.enums import Arms, Grasp, GripperState, ObjectType
from ..designator import ActionDesignatorDescription
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..ontology.ontology import OntologyConceptHolder

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

from ..utils import translate_relative_to_object


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    def __init__(self, positions: List[float], resolver=None,
                 ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional specialized_designators that returns a performable designator for a designator description.
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.positions: List[float] = positions

        if self.soma:
            self.init_ontology_concepts({"move_torso": self.soma.MoveTorso})

    def ground(self) -> MoveTorsoActionPerformable:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return MoveTorsoActionPerformable(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield MoveTorsoActionPerformable(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    def __init__(self, grippers: List[Arms], motions: List[GripperState], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative specialized_designators that returns a performable designator for a designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[GripperState] = grippers
        self.motions: List[Arms] = motions

        if self.soma:
            self.init_ontology_concepts({"setting_gripper": self.soma.SettingGripper})

    def ground(self) -> SetGripperActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return SetGripperActionPerformable(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield SetGripperActionPerformable(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[Arms], object_designator_description: ObjectDesignatorDescription,
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[Arms] = grippers
        self.object_designator_description = object_designator_description

        if self.soma:
            self.init_ontology_concepts({"releasing": self.soma.Releasing})

    def ground(self) -> ReleaseActionPerformable:
        return ReleaseActionPerformable(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[Arms], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[Arms] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

        if self.soma:
            self.init_ontology_concepts({"holding": self.soma.Holding})

    def ground(self) -> GripActionPerformable:
        return GripActionPerformable(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    def __init__(self, arms: List[Arms], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional specialized_designators that returns a performable designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"parking_arms": self.soma.ParkingArms})

    def ground(self) -> ParkArmsActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return ParkArmsActionPerformable(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms], grasps: List[Grasp], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional specialized_designators that returns a performable designator with elements from the lists of possible paramter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.grasps: List[Grasp] = grasps

        if self.soma:
            self.init_ontology_concepts({"picking_up": self.soma.PickingUp})

    def ground(self) -> PickUpActionPerformable:
        """
        Default specialized_designators, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object):
            obj_desig = self.object_designator_description
        else:
            obj_desig = self.object_designator_description.resolve()

        return PickUpActionPerformable(obj_desig, self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[Arms], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.target_locations: List[Pose] = target_locations
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"placing": self.soma.Placing})

    def ground(self) -> PlaceActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    def __init__(self, target_locations: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative specialized_designators that creates a performable designator from the list of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.target_locations: List[Pose] = target_locations

        if self.soma:
            self.init_ontology_concepts({"navigating": self.soma.Navigating})

    def ground(self) -> NavigateActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return NavigateActionPerformable(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms],
                 target_locations: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative specialized_designators that returns a performable designator for the list of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.target_locations: List[Pose] = target_locations

        if self.soma:
            self.init_ontology_concepts({"transporting": self.soma.Transporting})

    def ground(self) -> TransportActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()

        return TransportActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    def __init__(self, targets: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative specialized_designators that returns a performable designator for a list of possible target locations
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.targets: List[Pose] = targets

        if self.soma:
            self.init_ontology_concepts({"looking_at": self.soma.LookingAt})

    def ground(self) -> LookAtActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return LookAtActionPerformable(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    def __init__(self, object_designator_description: ObjectDesignatorDescription, resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative specialized_designators
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description

        if self.soma:
            self.init_ontology_concepts({"looking_for": self.soma.LookingFor,
                                         "checking_object_presence": self.soma.CheckingObjectPresence})

    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        return DetectActionPerformable(self.object_designator_description.resolve())


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container-like object using a designated arm and navigation poses.

    This class handles the action of moving a robot's arm to interact with and open
    container-like objects. It specifies both the arm(s) used and the start and goal
    locations necessary for positioning the robot during the action.
    """

    object_designator_description: ObjectPart
    """
    Describes the specific part or handle of the object used for opening.
    """

    arms: List[Arms]
    """
    A list of possible arms designated for the opening action.
    """

    start_location: AccessingLocation.Location
    """
    The start location for the navigation pose required to perform the action.
    """

    goal_location: Optional[AccessingLocation.Location]
    """
    The goal location for the navigation pose, if different from the start location.
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms],
                 start_goal_location: List[AccessingLocation.Location],
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Initializes the OpenAction with specified object, arm, and location parameters.

        Args:
            object_designator_description (ObjectPart): Describes the part or handle of
                the object to be used for opening.
            arms (List[Arms]): A list of arms available for the action.
            start_goal_location (List[AccessingLocation.Location]): A list containing
                the start and goal locations for the navigation poses required to perform
                the action. If both entries are the same, only the start location is set.
            resolver (optional): A specialized designator that returns a performable
                designator for the list of possible parameters.
            ontology_concept_holders (Optional[List[Thing]]): Ontology concepts categorizing
                or associating the action with certain semantic meanings.
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.start_location: AccessingLocation.Location = start_goal_location[0]
        self.goal_location: AccessingLocation.Location = None if start_goal_location[0] == start_goal_location[1] else \
            start_goal_location[1]

        if self.soma:
            self.init_ontology_concepts({"opening": self.soma.Opening})

    def ground(self) -> OpenActionPerformable:
        """
        Returns a performable designator for the `OpenAction`, using the resolved object
        description and the primary entries from the provided parameter lists.

        This method creates an `OpenActionPerformable` instance based on the resolved
        description of the target object, the first available arm, and the predefined
        start and goal locations for executing the action.

        Returns:
            OpenActionPerformable: An instance representing a performable designator for the
            action, ready to be executed with the specified parameters.
        """
        return OpenActionPerformable(
            self.object_designator_description.resolve(),
            self.arms[0],
            self.start_location,
            self.goal_location
        )


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container-like object using a designated arm and navigation poses.

    This class manages the action of moving a robot's arm to interact with and close
    container-like objects. It specifies the arm(s) to use and the start and goal
    locations required for positioning the robot during the action.
    """

    object_designator_description: ObjectPart
    """
    Describes the specific part or handle of the object used for closing.
    """

    arms: List[Arms]
    """
    A list of possible arms designated for the closing action.
    """

    start_location: AccessingLocation.Location
    """
    The start location for the navigation pose required to perform the action.
    """

    goal_location: Optional[AccessingLocation.Location]
    """
    The goal location for the navigation pose, if different from the start location.
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms],
                 start_goal_location: List[AccessingLocation.Location],
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Initializes the CloseAction with specified object, arm, and location parameters.

        Args:
            object_designator_description (ObjectPart): Describes the part or handle of
                the object to be used for closing.
            arms (List[Arms]): A list of arms available for the action.
            start_goal_location (List[AccessingLocation.Location]): A list containing
                the start and goal locations for the navigation poses required to perform
                the action. If both entries are the same, only the start location is set.
            resolver (optional): A specialized designator that returns a performable
                designator for the list of possible parameters.
            ontology_concept_holders (Optional[List[Thing]]): Ontology concepts categorizing
                or associating the action with certain semantic meanings.
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.start_location: AccessingLocation.Location = start_goal_location[0]
        self.goal_location: AccessingLocation.Location = None if start_goal_location[0] == start_goal_location[1] else \
            start_goal_location[1]

        if self.soma:
            self.init_ontology_concepts({"closing": self.soma.Closing})

    def ground(self) -> CloseActionPerformable:
        """
        Returns a performable designator for the `CloseAction`, using the resolved object
        designator and the primary entry from the list of possible arms.

        This method creates a `CloseActionPerformable` instance based on the resolved
        description of the target object, the first available arm, and the predefined
        start and goal locations for executing the action.

        Returns:
            CloseActionPerformable: An instance representing a performable designator for
            the action, ready to be executed with the specified parameters.
        """
        return CloseActionPerformable(
            self.object_designator_description.resolve(),
            self.arms[0],
            self.start_location,
            self.goal_location
        )


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    def __init__(self, arms: List[Arms], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative specialized_designators to get a specified designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.arms: List[Arms] = arms
        self.object_description: ObjectDesignatorDescription = object_description

        if self.soma:
            self.init_ontology_concepts({"grasping": self.soma.Grasping})

    def ground(self) -> GraspingActionPerformable:
        """
        Default specialized_designators that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return GraspingActionPerformable(self.arms[0], self.object_description.resolve())


class CuttingAction(ActionDesignatorDescription):
    """
    A designator for robotic cutting actions. This class facilitates the specification and execution of
    cutting tasks using a robot.
    """

    def __init__(self, object_to_be_cut: ObjectDesignatorDescription, tool: ObjectDesignatorDescription,
                 arms: List[Arms], technique: Optional[str] = None, slice_thickness: Optional[float] = 0.03):
        """
        Initializes a CuttingAction with specified object and tool designators, arms, and an optional cutting technique.

        Args:
            object_to_be_cut: Designator for the object to be cut.
            tool: Designator for the cutting tool.
            arms: List of possible arms to be used for the cutting action.
            technique: Optional cutting technique to be used.
        """
        super(CuttingAction, self).__init__()
        self.object_to_be_cut: ObjectDesignatorDescription = object_to_be_cut
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[Arms] = arms
        self.technique: Optional[str] = technique
        self.slice_thickness: Optional[float] = slice_thickness

    def ground(self) -> CuttingPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return CuttingPerformable(self.object_to_be_cut, self.tool, self.arms[0], self.technique,
                                  self.slice_thickness)


class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a mixing action.
    """

    def __init__(self, object_: ObjectDesignatorDescription, tool: ObjectDesignatorDescription,
                 arms: List[Arms], technique: Optional[str] = None):
        """
        """
        super(MixingAction, self).__init__()
        self.object_: ObjectDesignatorDescription = object_
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[Arms] = arms
        self.technique: Optional[str] = technique

    def ground(self) -> MixingPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return MixingPerformable(self.object_, self.tool, self.arms[0], self.technique)


class PouringAction(ActionDesignatorDescription):
    """
    Designator for pouring liquids from one container to another.
    """

    def __init__(self, object_: ObjectDesignatorDescription, tool: ObjectDesignatorDescription,
                 arms: List[Arms], technique: Optional[str] = None, angle: Optional[float] = 0.03):
        """

        """
        super(PouringAction, self).__init__()
        self.object_: ObjectDesignatorDescription = object_
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[Arms] = arms
        self.technique: Optional[str] = technique
        self.angle: Optional[float] = angle

    def ground(self) -> PouringPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return PouringPerformable(self.object_, self.tool, self.arms[0], self.technique, self.angle)


# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable performables."""
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
class MixingPerformable(ActionAbstract):
    """
        Action class for the Mixing action.
        """

    object_: ObjectDesignatorDescription
    """
    The object to be cut.
    """

    tool: ObjectDesignatorDescription
    """
    The tool used for cutting.
    """

    arm: Arms
    """
    The robot arm designated for the cutting task.
    """

    technique: Optional[str] = None
    """
    The technique used for cutting (default is None).
    """

    @with_tree
    def perform(self) -> None:
        """
            Perform the mixing action using the specified object, tool, arm, and grasp.
            """
        # Retrieve object and robot from designators
        local_tf = LocalTransformer()
        obj = self.object_.world_object

        obj_dim = obj.get_object_dimensions()

        dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
        obj_height = dim[2]
        oTm = self.object_.pose
        object_pose = local_tf.transform_to_object_frame(oTm, obj)

        def generate_spiral(pose, upward_increment, radial_increment, angle_increment, steps):
            x_start, y_start, z_start = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            spiral_poses = []

            for t in range(2 * steps):
                tmp_pose = pose.copy()

                r = radial_increment * t
                a = angle_increment * t
                h = upward_increment * t

                x = x_start + r * math.cos(a)
                y = y_start + r * math.sin(a)
                z = z_start + h

                tmp_pose.pose.position.x += x
                tmp_pose.pose.position.y += y
                tmp_pose.pose.position.z += z

                spiralTm = local_tf.transform_pose(tmp_pose, "map")
                spiral_poses.append(spiralTm)
                World.current_world.add_vis_axis(spiralTm)
                spiralTm.pose.position.z += obj_height
                MoveTCPMotion(spiralTm, self.arm).perform()
            return spiral_poses

        # this is a very good one but takes ages
        # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
        spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)

        World.current_world.remove_vis_axis()


@dataclass
class PouringPerformable(ActionAbstract):
    """
    Action class for the Pouring action.
    """

    object_: ObjectDesignatorDescription
    """
    The object to be cut.
    """

    tool: ObjectDesignatorDescription
    """
    The tool used for cutting.
    """

    arm: Arms
    """
    The robot arm designated for the cutting task.
    """

    technique: Optional[str] = None
    """
    The technique used for cutting (default is None).
    """

    angle: Optional[float] = 90
    """
    """

    @with_tree
    def perform(self) -> None:
        # Initialize the local transformer and robot reference
        lt = LocalTransformer()

        # Calculate the object's pose in the map frame
        oTm = self.object_.pose
        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.grasps[
            Grasp.FRONT]

        oTbs = lt.transform_pose(oTm, World.robot.get_link_tf_frame("base_link"))
        oTbs.pose.position.x += 0.009  # was 0,009
        oTbs.pose.position.z += 0.17  # was 0.13
        oTbs.pose.position.y -= 0.125
        # if self.direction == Grasp.RIGHT:
        #     oTbs.pose.position.y -= 0.125
        # else:
        #     oTbs.pose.position.y += 0.125

        oTms = lt.transform_pose(oTbs, "map")
        World.current_world.add_vis_axis(oTms)

        #
        oTog = lt.transform_pose(oTms, World.robot.get_link_tf_frame("base_link"))
        oTog.orientation = grasp_rotation
        self.angle = 75  # for a 90-degree rotation

        oTgm = lt.transform_pose(oTog, "map")
        MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).perform()

        World.current_world.add_vis_axis(oTgm)
        adjusted_oTgm = oTgm.copy()

        new_q = utils.axis_angle_to_quaternion([1, 0, 0], self.angle)
        # Multiply the quaternions to combine rotations
        adjusted_oTgm.multiply_quaternions(new_q)

        # if self.direction == "right":
        #     new_q = utils.axis_angle_to_quaternion([0, 0, 1], -self.angle)
        # else:
        #     new_q = utils.axis_angle_to_quaternion([0, 0, 1], self.angle)

        World.current_world.add_vis_axis(adjusted_oTgm)
        MoveTCPMotion(adjusted_oTgm, self.arm, allow_gripper_collision=False).perform()


@dataclass
class CuttingPerformable(ActionAbstract):
    """
        Represents a specific cutting action to be performed by the robot. This class encapsulates
        all necessary details for executing the cutting task, including the object to be cut, the cutting tool,
        the arm to use, and the cutting technique.
        """

    object_to_be_cut: ObjectDesignatorDescription
    """
    The object to be cut.
    """

    tool: ObjectDesignatorDescription
    """
    The tool used for cutting.
    """

    arm: Arms
    """
    The robot arm designated for the cutting task.
    """

    technique: Optional[str] = None
    """
    The technique used for cutting (default is None).
    """

    slice_thickness: Optional[float] = 0.03
    """
    The upper bound thickness of the slices (default is 0.03f).
    """

    def calculate_slices(self, obj_length, technique, thickness):
        """
        Calculates the number of slices and the starting offset based on the cutting technique and slice thickness.
        """
        # slicing
        num_slices = int(obj_length // thickness)
        start_offset = (-obj_length / 2) + (thickness / 2)

        # Calculate the starting position for slicing, adjusted based on the chosen technique
        if technique in ['Halving']:
            start_offset = 0  # No offset needed for halving
            num_slices = 1  # Only one slice for halving
            return num_slices, start_offset
        elif technique in ['Cutting Action', 'Sawing', 'Paring', 'Cutting', 'Carving']:
            # Calculate number of slices and initial offset for regular slicing
            num_slices = 1

        return num_slices, start_offset

    def calculate_slice_pose(self, object_pose, slice_position, obj_width):
        """
            Determines the pose for each slice based on the object's current pose, the slice position, and the object's dimensions.
            """
        slice_pose = object_pose.copy()
        slice_pose.position.x = slice_position

        return slice_pose

    def adjust_for_lifting(self, pose, height):
        """
            Adjusts the given pose to lift the cutting tool above the object before and after cutting.
            """
        lift_pose = pose.copy()
        lift_pose.position.z += 2 * height  # Lift the tool above the object
        return lift_pose

    def facing_robot(self, pose):
        """
            Checks if the object is facing the robot.
            """

        rotate_q = utils.axis_angle_to_quaternion([0, 0, 1], 180)
        pose.multiply_quaternions(rotate_q)
        return pose

    def perpendicular_pose(self, pose):
        perpendicular_pose = pose.copy()
        rotation_axis = [0, 0, 1]
        rotation_quaternion = utils.axis_angle_to_quaternion(rotation_axis, 90)
        perpendicular_pose.multiply_quaternions(rotation_quaternion)
        return perpendicular_pose

    @with_tree
    def perform(self) -> None:
        local_tf = LocalTransformer()
        # Access the designated object for cutting and retrieve its dimensions
        obj = self.object_to_be_cut.world_object
        obj_length, obj_width, object_height = obj.get_object_dimensions()

        # Retrieve the current pose of the object and transform it to the object frame
        oTm = self.object_to_be_cut.pose
        # BulletWorld.current_bullet_world.add_vis_axis(oTm)
        object_pose = local_tf.transform_to_object_frame(oTm, obj)

        num_slices, start_offset = self.calculate_slices(obj_length, self.technique, self.slice_thickness)

        # Generate coordinates for each slice along the object's length
        slice_coordinates = [start_offset + i * self.slice_thickness for i in range(num_slices)]

        # Transform the coordinates of each slice to the map frame, maintaining orientation
        slice_poses = []
        for x in slice_coordinates:
            # Adjust slice pose based on object dimensions and orientation
            tmp_pose = object_pose.copy()
            tmp_pose.pose.position.y += obj_width  # plus tool länge  # Offset position for slicing
            tmp_pose.pose.position.x = x  # Set slicing position
            # tmp_pose.pose.position.z
            sTm = local_tf.transform_pose(tmp_pose, "map")
            # BulletWorld.current_bullet_world.add_vis_axis(sTm)
            slice_poses.append(sTm)

        # Process each slice pose for the cutting action
        for slice_pose in slice_poses:
            # check if obj is facing the object
            slice_pose = self.facing_robot(slice_pose)

            perpendicular_pose = self.perpendicular_pose(slice_pose)

            object_pose = self.tool.pose

            # Transformations such that the target position is the position of the object and not the tcp
            gripper_name = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
            tcp_to_object = local_tf.transform_pose(object_pose,
                                                    World.robot.get_link_tf_frame(gripper_name))
            target_diff = (perpendicular_pose.to_transform("target").inverse_times(
                tcp_to_object.to_transform("object")).to_pose())

            lift_pose = target_diff.copy()
            lift_pose.pose.position.z += object_height  # Lift the tool above the object
            World.current_world.add_vis_axis(target_diff)
            # BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
            MoveTCPMotion(lift_pose, self.arm).perform()
            MoveTCPMotion(target_diff, self.arm).perform()
            MoveTCPMotion(lift_pose, self.arm).perform()


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
        torso_joint = RobotDescription.current_robot_description.get_torso_joint()
        MoveJointsMotion([torso_joint], [self.position]).perform()


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
    def perform(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: Arms

    object_designator: ObjectDesignatorDescription.Object

    def perform(self) -> None:
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
            arm_chain = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT)
            left_poses = arm_chain.get_static_joint_states(kwargs["left_arm_config"]) if arm_chain else None

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            arm_chain = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT)
            right_poses = arm_chain.get_static_joint_states(kwargs["right_arm_config"]) if arm_chain else None

        MoveArmJointsMotion(left_poses, right_poses).perform()


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be picked up.
    """

    arm: Arms
    """
    The arm enum that should be used for pick up, e.g., Arms.LEFT or Arms.RIGHT.
    """

    grasp: Grasp
    """
    The grasp enum that should be used, e.g., Grasp.FRONT or Grasp.RIGHT.
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
    """
    The object at the time this action was created. Acts as a static information holder and 
    is not updated when the BulletWorld object is changed.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)
    """
    ORM class type for this action.
    """
    @with_tree
    def perform(self) -> None:
        """
        Executes the action to pick up the designated object with specified parameters,
        moving the robot arm to the pre-pick position, grasping, and lifting the object.
        """
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.frozen_copy()
        robot = World.robot
        # Retrieve object and robot from designators
        object = self.object_designator.world_object
        # oTm = Object Pose in Frame map
        oTm = object.get_pose()

        # Adjust the pose according to the special knowledge of the object designator
        # I think this should be irrelevant now due to the grasp_offset calculation, but not sure, so just commented out for now
        # adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
        adjusted_grasp = adjust_grasp_for_object_rotation(oTm, self.grasp, self.arm)
        adjusted_oTm = oTm.copy()
        adjusted_oTm.set_orientation(adjusted_grasp)

        grasp_offset = calculate_grasp_offset(object.get_object_dimensions(), self.arm, self.grasp)
        palm_axis = RobotDescription.current_robot_description.get_palm_axis()
        adjusted_oTm_grasp_pose = translate_relative_to_object(adjusted_oTm, palm_axis, grasp_offset)

        translation_value = 0.1  # hardcoded value for now
        oTm_prepose = translate_relative_to_object(adjusted_oTm_grasp_pose, palm_axis, translation_value)
        prepose = object.local_transformer.transform_pose(oTm_prepose, "map")

        if World.current_world.allow_publish_debug_poses:
            gripper_pose = World.robot.get_link_pose(
                RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame())
            marker = AxisMarkerPublisher()
            marker.publish([adjusted_oTm_grasp_pose, prepose, gripper_pose], length=0.3)

        MoveTCPMotion(prepose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        MoveTCPMotion(adjusted_oTm_grasp_pose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm).perform()
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        robot.attach(object, tool_frame)

        adjusted_oTm_grasp_pose.pose.position.z += 0.03
        MoveTCPMotion(adjusted_oTm_grasp_pose, self.arm, allow_gripper_collision=True).perform()


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an object at a specified position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Describes the object that should be placed.
    """

    arm: Arms
    """
    The arm currently holding the object.
    """

    target_location: Pose
    """
    The world pose at which the object should be placed.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)
    """
    ORM class type for this action.
    """

    @with_tree
    def perform(self) -> None:
        """
        Executes the place action, positioning the arm to the specified target location,
        releasing the object, and retracting the arm.
        """
        object_pose = self.object_designator.world_object.get_pose()
        local_tf = LocalTransformer()

        # Transformations such that the target position is the position of the object and not the tcp
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                World.robot.get_link_tf_frame(
                                                    RobotDescription.current_robot_description.get_arm_chain(
                                                        self.arm).get_tool_frame()))
        target_diff = self.target_location.to_transform("target").inverse_times(
            tcp_to_object.to_transform("object")).to_pose()

        if World.current_world.allow_publish_debug_poses:
            gripper_pose = World.robot.get_link_pose(
                RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame())
            marker = AxisMarkerPublisher()
            marker.publish([target_diff, self.target_location, gripper_pose], length=0.3)

        MoveTCPMotion(target_diff, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        World.robot.detach(self.object_designator.world_object)

        retract_pose = local_tf.transform_pose(target_diff, World.robot.get_link_tf_frame(
            RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()))

        palm_axis = RobotDescription.current_robot_description.get_palm_axis()
        translation_value = 0.1
        retract_pose = translate_relative_to_object(retract_pose, palm_axis, translation_value)

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
    Transports an object to a specified position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Describes the object that should be transported.
    """

    arm: Arms
    """
    The arm designated for transporting the object.
    """

    target_location: Pose
    """
    The target world pose to which the object should be transported.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)
    """
    ORM class type for this action.
    """

    @with_tree
    def perform(self) -> None:
        """
        Executes the transport action by navigating to the pick-up and target locations
        to grasp, transport, and place the designated object.

        The process includes:
        1. Parking both arms for initial positioning.
        2. Determining the appropriate grasp based on object type.
        3. Finding a reachable pose for the robot to pick up the object with the designated arm.
        4. Navigating to the pick-up pose, performing the pick-up, and then parking arms again.
        5. Resolving a place location reachable by the robot to place the object.
        6. Navigating to the place location, performing the placement, and parking arms post-transport.

        Raises:
            ObjectUnfetchable: If no reachable pose is found for the pick-up.
            ReachabilityFailure: If no reachable location is found for the target location.
        """
        robot_desig = BelieveObject(names=[RobotDescription.current_robot_description.name])
        ParkArmsActionPerformable(Arms.BOTH).perform()

        if self.object_designator.obj_type == ObjectType.BOWL or self.object_designator.obj_type == ObjectType.SPOON:
            grasp = calculate_object_faces(self.object_designator)[1]
        else:
            grasp = calculate_object_faces(self.object_designator)[0]

        pickup_loc = CostmapLocation(
            target=self.object_designator,
            reachable_for=robot_desig.resolve(),
            reachable_arm=self.arm,
            used_grasps=[grasp]
        )

        # Tries to find a pick-up posotion for the robot that uses the given arm
        pickup_pose = next((pose for pose in pickup_loc if self.arm in pose.reachable_arms), None)
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"No reachable pose found for the robot to grasp the object: {self.object_designator} with arm: {self.arm}"
            )

        NavigateActionPerformable(pickup_pose.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, grasp).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(
                target=self.target_location,
                reachable_for=robot_desig.resolve(),
                reachable_arm=self.arm,
                used_grasps=[grasp],
                object_in_hand=self.object_designator
            ).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No reachable location found for the target location: {self.target_location}"
            )
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

        marker = AxisMarkerPublisher()
        camera_link = RobotDescription.current_robot_description.get_camera_frame()
        camera_pose = World.robot.get_link_pose(camera_link)
        marker.publish([self.target, camera_pose], name="Looking", length=0.3)


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
        return DetectingMotion(object_type=self.object_designator.obj_type).perform()


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container-like object using a specified arm.
    """

    object_designator: ObjectPart.Object
    """
    Describes the object that should be opened.
    """

    arm: Arms
    """
    The arm designated for opening the container.
    """

    start_location: AccessingLocation.Location
    """
    The initial location from which the opening action begins.
    """

    goal_location: Optional[AccessingLocation.Location]
    """
    The final goal location for the opening action.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)
    """
    ORM class type for this action.
    """

    @with_tree
    def perform(self) -> None:
        """
        Executes the open action by navigating to the start location, performing a grasp,
        and executing the opening motion to the goal location.

        The process includes:
        1. Navigating to the specified `start_location`.
        2. Performing the grasp action on the designated object.
        3. Executing the opening motion towards the `goal_location`.
        4. Releasing the gripper to complete the opening.

        """
        NavigateAction([self.start_location.pose]).resolve().perform()
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm, self.goal_location).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container-like object using a specified arm.
    """

    object_designator: ObjectPart.Object
    """
    Describes the object that should be closed.
    """

    arm: Arms
    """
    The arm designated for closing the container.
    """

    start_location: AccessingLocation.Location
    """
    The initial location from which the closing action begins.
    """

    goal_location: Optional[AccessingLocation.Location]
    """
    The final goal location for the closing action.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)
    """
    ORM class type for this action.
    """

    @with_tree
    def perform(self) -> None:
        """
        Executes the close action by navigating to the start location, performing a grasp,
        and executing the closing motion to the goal location.

        The process includes:
        1. Navigating to the specified `start_location`.
        2. Performing the grasp action on the designated object.
        3. Executing the closing motion towards the `goal_location`.
        4. Releasing the gripper to complete the closing.

        """
        NavigateAction([self.start_location.pose]).resolve().perform()
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm, self.goal_location).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object as described by the given object designator.
    """

    arm: Arms
    """
    The arm to be used for grasping the object.
    """

    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object designator for the object that should be grasped.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)
    """
    ORM class type for this action.
    """

    @with_tree
    def perform(self) -> None:
        """
        Executes the grasp action by navigating to a pre-grasp position and grasping the object.

        The sequence includes:
        1. Determining the object's pose and adjusting the grasp orientation.
        2. Translating to a pre-grasp position.
        3. Moving the robot's arm to the pre-grasp position and opening the gripper.
        4. Moving to the grasp position and closing the gripper to secure the object.

        Notes:
            - The grasp side is currently hardcoded to `Grasp.FRONT`.
            - This approach adjusts the pose orientation to align with the grasp angle.
        """
        if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig.part_pose
        else:
            object_pose = self.object_desig.world_object.get_pose()

        # TODO: there is a difference in which side faces the object during costmap and execution, so hardcoded for now, fix it
        # grasp = calculate_object_faces(self.object_desig)[0]
        grasp = Grasp.FRONT

        adjusted_grasp = adjust_grasp_for_object_rotation(object_pose, grasp, self.arm)
        object_pose = object_pose.copy()
        object_pose.set_orientation(adjusted_grasp)

        palm_axis = RobotDescription.current_robot_description.get_palm_axis()
        translation_value = 0.05
        local_transformer = LocalTransformer()
        oTm_prepose = translate_relative_to_object(object_pose, palm_axis, translation_value)

        prepose = local_transformer.transform_pose(oTm_prepose, "map")

        if World.current_world.allow_publish_debug_poses:
            marker = AxisMarkerPublisher()
            marker.publish([object_pose, prepose], name="Grasping", length=0.3)

        MoveTCPMotion(prepose, self.arm).perform()
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
    def perform(self) -> None:
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

    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()
