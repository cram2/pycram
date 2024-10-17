# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import typing_extensions
from dataclasses import dataclass, field, fields
from abc import ABC, abstractmethod
from inspect import isgenerator, isgeneratorfunction

from .ros.logging import logwarn, loginfo

import inspect

from .knowledge.knowledge_engine import KnowledgeEngine

try:
    import owlready2
except ImportError:
    owlready2 = None
    logwarn("owlready2 is not installed!")

from sqlalchemy.orm.session import Session

from .datastructures.world import World
from .world_concepts.world_object import Object as WorldObject
from .utils import GeneratorList, bcolors
from threading import Lock
from time import time
from typing_extensions import Type, List, Dict, Any, Optional, Union, get_type_hints, Callable, Iterable, TYPE_CHECKING, get_args, get_origin

from .local_transformer import LocalTransformer
from .language import Language
from .datastructures.pose import Pose
from .robot_description import RobotDescription
from .datastructures.enums import ObjectType, Grasp

import logging

from .orm.action_designator import (Action as ORMAction)
from .orm.object_designator import (Object as ORMObjectDesignator)
from .orm.motion_designator import Motion as ORMMotionDesignator

from .orm.base import RobotState, ProcessMetaData
from .tasktree import with_tree

if TYPE_CHECKING:
    from .ontology.ontology_common import OntologyConceptHolder


class DesignatorError(Exception):
    """Implementation of designator_description errors."""

    def __init__(self, *args, **kwargs):
        """Create a new designator_description error."""
        Exception.__init__(self, *args, **kwargs)


class ResolutionError(Exception):
    def __init__(self, missing_properties: List[str], wrong_type: Dict, current_type: Any,
                 designator: DesignatorDescription):
        self.error = f"\nSome requiered properties where missing or had the wrong type when grounding the Designator: {designator}.\n"
        self.missing = f"The missing properties where: {missing_properties}\n"
        self.wrong = f"The properties with the wrong type along with the currrent -and right type :\n"
        self.head = "Property   |   Current Type    |     Right Type\n-------------------------------------------------------------\n"
        self.tab = ""
        for prop in wrong_type.keys():
            self.tab += prop + "     " + str(current_type[prop]) + "      " + str(wrong_type[prop]) + "\n"
        self.message = self.error
        if missing_properties != []:
            self.message += self.missing
        if wrong_type != {}:
            self.message += self.wrong + self.head + self.tab
        self.message = f"{bcolors.BOLD}{bcolors.FAIL}" + self.message + f"{bcolors.ENDC}"
        super(ResolutionError, self).__init__(self.message)


class DesignatorDescription(ABC):
    """
    :ivar resolve: The specialized_designators function to use for this designator_description, defaults to self.ground
    """

    def __init__(self, ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Create a Designator description.

        :param ontology_concept_holders: A list of holders of ontology concepts that the designator_description is categorized as or associated with
        """

        # self.resolve = self.ground
        self.ontology_concept_holders = [] if ontology_concept_holders is None else ontology_concept_holders

    def resolve(self):
        return self.ground()

    def make_dictionary(self, properties: List[str]):
        """
        Creates a dictionary of this description with only the given properties
        included.

        :param properties: A list of properties that should be included in the dictionary.
                            The given properties have to be an attribute of this description.
        :return: A dictionary with the properties as keys.
        """
        attributes = self.__dict__
        ret = {}
        for att in attributes.keys():
            if att in properties:
                ret[att] = attributes[att]
        return ret

    def ground(self) -> Any:
        """
        Should be overwritten with an actual grounding function which infers missing properties.
        """
        return self

    def get_slots(self) -> List[str]:
        """
        :return: a list of all slots of this description. Can be used for inspecting different descriptions and
         debugging.
        """
        return list(self.__dict__.keys())

    def copy(self) -> DesignatorDescription:
        return self

    def get_default_ontology_concept(self) -> owlready2.Thing | None:
        """
        :return: The first element of ontology_concept_holders if there is, else None
        """
        return self.ontology_concept_holders[0].ontology_concept if self.ontology_concept_holders else None

    def get_optional_parameter(self) -> List[str]:
        """
        Returns a list of optional parameter names of this designator_description description.
        """
        return [param_name for param_name, param in inspect.signature(self.__init__).parameters.items() if
                param.default != param.empty]

    def get_all_parameter(self) -> List[str]:
        """
        Returns a list of all parameter names of this designator_description description.
        """
        return [param_name for param_name, param in inspect.signature(self.__init__).parameters.items()]

    def get_type_hints(self) -> Dict[str, Any]:
        """
        Returns the type hints of the __init__ method of this designator_description description.

        :return:
        """
        return typing_extensions.get_type_hints(self.__init__)

class ActionDesignatorDescription(DesignatorDescription, Language):
    """
    Abstract class for action designator_description descriptions.
    Descriptions hold possible parameter ranges for action designators.
    """

    knowledge_condition = None
    """
    Knowledge condition that have to be fulfilled before executing the action.
    """

    performable_class: Type[ActionDesignatorDescription.Action]
    """
    Reference to the performable class that is used to execute the action.
    """

    @dataclass
    class Action:
        """
        The performable designator_description with a single element for each list of possible parameter.
        """
        robot_position: Pose = field(init=False)
        """
        The position of the robot at the start of the action.
        """
        robot_torso_height: float = field(init=False)
        """
        The torso height of the robot at the start of the action.
        """

        robot_type: ObjectType = field(init=False)
        """
        The type of the robot at the start of the action.
        """

        def __post_init__(self):
            self.robot_position = World.robot.get_pose()
            self.robot_torso_height = World.robot.get_joint_position(
                RobotDescription.current_robot_description.torso_joint)
            self.robot_type = World.robot.obj_type

        def perform(self) -> Any:
            """
            Executes the action with the single parameters from the description.

            :return: The result of the action in the plan
            """
            self.pre_perform()
            result = self.plan()
            self.post_perform()
            return result

        @with_tree
        def plan(self) -> Any:
            """
            Plan of the action. To be overridden by subclasses.

            :return: The result of the action, if there is any
            """
            raise NotImplementedError()

        def pre_perform(self):
            """
            This method is called before the perform method is executed. To be overridden by subclasses.
            """
            pass

        def post_perform(self):
            """
            This method is called after the perform method is executed. To be overridden by subclasses.
            """
            pass

        def to_sql(self) -> ORMAction:
            """
            Create an ORM object that corresponds to this description.

            :return: The created ORM object.
            """
            raise NotImplementedError(f"{type(self)} has no implementation of to_sql. Feel free to implement it.")

        def insert(self, session: Session, *args, **kwargs) -> ORMAction:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :param args: Possible extra arguments
            :param kwargs: Possible extra keyword arguments
            :return: The completely instanced ORM object
            """

            pose = self.robot_position.insert(session)

            # get or create metadata
            metadata = ProcessMetaData().insert(session)

            # create robot-state object
            robot_state = RobotState(self.robot_torso_height, self.robot_type)
            robot_state.pose = pose
            robot_state.process_metadata = metadata
            session.add(robot_state)

            # create action
            action = self.to_sql()
            action.process_metadata = metadata
            action.robot_state = robot_state

            return action

        @classmethod
        def get_type_hints(cls) -> Dict[str, Any]:
            """
            Returns the type hints of the __init__ method of this designator_description description.

            :return:
            """
            return typing_extensions.get_type_hints(cls)

    def __init__(self, ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Base of all action designator_description descriptions.

        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(ontology_concept_holders)
        Language.__init__(self)
        from .ontology.ontology import OntologyManager
        self.soma = OntologyManager().soma
        self.knowledge_conditions = None

    def resolve(self) -> Type[ActionDesignatorDescription.Action]:
        """
        Resolves this designator_description to a performable designtor by using the reasoning of the knowledge engine.
        This method will simply take the first result from iterating over the designator_description.

        :return: A fully specified Action Designator
        """
        if getattr(self, "__iter__", None):
            return next(iter(self))
        raise NotImplementedError(f"{type(self)} has no __iter__ method.")

    def ground(self) -> Action:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")

    def init_ontology_concepts(self, ontology_concept_classes: Dict[str, Type[owlready2.Thing]]):
        """
        Initialize the ontology concept holders for this action designator_description

        :param ontology_concept_classes: The ontology concept classes that the action is categorized as or associated with
        :param ontology_concept_name: The name of the ontology concept instance to be created
        """
        from .ontology.ontology_common import OntologyConceptHolderStore, OntologyConceptHolder
        if not self.ontology_concept_holders:
            for concept_name, concept_class in ontology_concept_classes.items():
                if concept_class:
                    existing_holders = OntologyConceptHolderStore().get_ontology_concept_holders_by_class(concept_class)
                    self.ontology_concept_holders.extend(existing_holders if existing_holders \
                                                             else [OntologyConceptHolder(concept_class(concept_name))])



class LocationDesignatorDescription(DesignatorDescription):
    """
    Parent class of location designator_description descriptions.
    """

    @dataclass
    class Location:
        """
        Resolved location that represents a specific point in the world which satisfies the constraints of the location
        designator_description description.
        """
        pose: Pose
        """
        The executed pose of the location designator_description. Pose is inherited by all location designator_description.
        """

    def __init__(self, ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        super().__init__(ontology_concept_holders)

    def ground(self) -> Location:
        """
        Find a location that satisfies all constrains.
        """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


# this knowledge should be somewhere else i guess
SPECIAL_KNOWLEDGE = {
    'bigknife':
        [("top", [-0.08, 0, 0])],
    'whisk':
        [("top", [-0.08, 0, 0])],
    'bowl':
        [("front", [1.0, 2.0, 3.0]),
         ("key2", [4.0, 5.0, 6.0])]
}


class ObjectDesignatorDescription(DesignatorDescription):
    """
    Class for object designator_description descriptions.
    Descriptions hold possible parameter ranges for object designators.
    """

    @dataclass
    class Object:
        """
        A single element that fits the description.
        """

        name: str
        """
        Name of the object
        """

        obj_type: ObjectType
        """
        Type of the object
        """

        world_object: Optional[WorldObject]
        """
        Reference to the World object
        """

        _pose: Optional[Callable] = field(init=False)
        """
        A callable returning the pose of this object. The _pose member is used overwritten for data copies
        which will not update when the original world_object is moved.
        """

        def __post_init__(self):
            if self.world_object:
                self._pose = self.world_object.get_pose

        def to_sql(self) -> ORMObjectDesignator:
            """
            Create an ORM object that corresponds to this description.

            :return: The created ORM object.
            """
            return ORMObjectDesignator(name=self.name, obj_type=self.obj_type)

        def insert(self, session: Session) -> ORMObjectDesignator:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :return: The completely instanced ORM object
            """
            metadata = ProcessMetaData().insert(session)
            pose = self.pose.insert(session)

            # create object orm designator_description
            obj = self.to_sql()
            obj.process_metadata = metadata
            obj.pose = pose
            session.add(obj)
            return obj

        def frozen_copy(self) -> 'ObjectDesignatorDescription.Object':
            """
            Returns a copy of this designator_description containing only the fields.

            :return: A copy containing only the fields of this class. The WorldObject attached to this pycram object is not copied. The _pose gets set to a method that statically returns the pose of the object when this method was called.
            """
            result = ObjectDesignatorDescription.Object(self.name, self.obj_type, None)
            # get current object pose and set resulting pose to always be that
            pose = self.pose
            result.pose = lambda: pose
            return result

        @property
        def pose(self):
            """
            Property of the current position and orientation of the object.
            Evaluate the _pose function.

            :return: Position and orientation
            """
            return self._pose()

        @pose.setter
        def pose(self, value: Callable):
            """
            Set the pose to a new method that returns the current pose.

            :param value: A callable that returns a pose.
            """
            self._pose = value

        def __repr__(self):
            return self.__class__.__qualname__ + f"(" + ', '.join(
                [f"{f.name}={self.__getattribute__(f.name)}" for f in fields(self)] + [
                    f"pose={self.pose}"]) + ')'

        def special_knowledge_adjustment_pose(self, grasp: Grasp, pose: Pose) -> Pose:
            """
            Get the adjusted target pose based on special knowledge for "grasp front".

            :param grasp: From which side the object should be grasped
            :param pose: Pose at which the object should be grasped, before adjustment
            :return: The adjusted grasp pose
            """
            lt = LocalTransformer()
            pose_in_object = lt.transform_pose(pose, self.world_object.tf_frame)

            special_knowledge = []  # Initialize as an empty list
            if self.obj_type in SPECIAL_KNOWLEDGE:
                special_knowledge = SPECIAL_KNOWLEDGE[self.obj_type]

            for key, value in special_knowledge:
                if key == grasp:
                    # Adjust target pose based on special knowledge
                    pose_in_object.pose.position.x += value[0]
                    pose_in_object.pose.position.y += value[1]
                    pose_in_object.pose.position.z += value[2]
                    loginfo("Adjusted target pose based on special knowledge for grasp: %s", grasp)
                    return pose_in_object
            return pose

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[ObjectType]] = None,
                 ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        """
        Base of all object designator_description descriptions. Every object designator_description has the name and type of the object.

        :param names: A list of names that could describe the object
        :param types: A list of types that could represent the object
        :param ontology_concept_holders: A list of ontology concepts that the object is categorized as or associated with
        """
        super().__init__(ontology_concept_holders)
        self.types: Optional[List[ObjectType]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> Union[Object, bool]:
        """
        Return the first object from the world that fits the description.

        :return: A executed object designator_description
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Object]:
        """
        Iterate through all possible objects fitting this description

        :yield: A executed object designator_description
        """
        # for every world object
        for obj in World.current_world.objects:

            # skip if name does not match specification
            if self.names and obj.name not in self.names:
                continue

            # skip if type does not match specification
            if self.types and obj.obj_type not in self.types:
                continue

            yield self.Object(obj.name, obj.obj_type, obj)

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

