# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from dataclasses import dataclass, field, fields
from abc import ABC, abstractmethod
from inspect import isgenerator, isgeneratorfunction

import rospy
try:
    import owlready2
except ImportError:
    owlready2 = None
    rospy.logwarn("owlready2 is not installed!")

from sqlalchemy.orm.session import Session

from .datastructures.world import World
from .world_concepts.world_object import Object as WorldObject
from .utils import GeneratorList, bcolors
from threading import Lock
from time import time
from typing_extensions import Type, List, Dict, Any, Optional, Union, get_type_hints, Callable, Iterable, TYPE_CHECKING

from .local_transformer import LocalTransformer
from .language import Language
from .datastructures.pose import Pose
from .robot_description import RobotDescription
from .datastructures.enums import ObjectType

import logging

from .orm.action_designator import (Action as ORMAction)
from .orm.object_designator import (Object as ORMObjectDesignator)

from .orm.base import RobotState, ProcessMetaData
from .tasktree import with_tree

if TYPE_CHECKING:
    from .ontology.ontology_common import OntologyConceptHolder


class DesignatorError(Exception):
    """Implementation of designator errors."""

    def __init__(self, *args, **kwargs):
        """Create a new designator error."""
        Exception.__init__(self, *args, **kwargs)


class ResolutionError(Exception):
    def __init__(self, missing_properties: List[str], wrong_type: Dict, current_type: Any, designator: Designator):
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


class Designator(ABC):
    """
    Implementation of designators. DEPRECTAED SINCE DESIGNATOR DESCRIPTIONS ARE USED AS BASE CLASS

    Designators are objects containing sequences of key-value pairs. They can be resolved which means to generate real
    parameters for executing performables from these pairs of key and value.

    :ivar timestamp: The timestamp of creation of reference or None if still not referencing an object.
    """


    resolvers = {}
    """
    List of all designator resolvers. Designator resolvers are functions which take a designator as
    argument and return a list of solutions. A solution can also be a generator. 
    """

    def __init__(self, description: DesignatorDescription, parent: Optional[Designator] = None):
        """Create a new desginator.

        Arguments:
        :param description: A list of tuples (key-value pairs) describing this designator.
        :param parent: The parent to equate with (default is None).
        """
        self._mutex: Lock = Lock()
        self._parent: Union[Designator, None] = None
        self._successor: Union[Designator, None] = None
        self._effective: bool = False
        self._data: Any = None
        self._solutions = None
        self._index: int = 0
        self.timestamp = None
        self._description: DesignatorDescription = description

        if parent is not None:
            self.equate(parent)

    def equate(self, parent: Designator) -> None:
        """Equate the designator with the given parent.

        Arguments:
        parent -- the parent to equate with.
        """
        if self.equal(parent):
            return

        youngest_parent = parent.current()
        first_parent = parent.first()

        if self._parent is not None:
            first_parent._parent = self._parent
            first_parent._parent._successor = first_parent

        self._parent = youngest_parent
        youngest_parent._successor = self

    def equal(self, other: Designator) -> bool:
        """Check if the designator describes the same entity as another designator, i.e. if they are equated.

        Arguments:
        other -- the other designator.
        """
        return other.first() is self.first()

    def first(self) -> Designator:
        """Return the first ancestor in the chain of equated designators."""
        if self._parent is None:
            return self

        return self._parent.first()

    def current(self) -> Designator:
        """Return the newest designator, i.e. that one that has been equated last to the designator or one of its
        equated designators."""
        if self._successor is None:
            return self

        return self._successor.current()

    def _reference(self) -> Any:
        """This is a helper method for internal usage only.

        This method is to be overwritten instead of the reference method.
        """
        resolver = self.resolvers[self._description.resolver]
        if self._solutions is None:
            def generator():
                solution = resolver(self)
                if isgeneratorfunction(solution):
                    solution = solution()

                if isgenerator(solution):
                    while True:
                        try:
                            yield next(solution)
                        except StopIteration:
                            break
                else:
                    yield solution

            self._solutions = GeneratorList(generator)

        if self._data is not None:
            return self._data

        try:
            self._data = self._solutions.get(self._index)
            return self._data
        except StopIteration:
            raise DesignatorError('There was no Solution for this Designator')

    def reference(self) -> Any:
        """Try to dereference the designator and return its data object or raise DesignatorError if it is not an
        effective designator. """
        with self._mutex:
            ret = self._reference()

        self._effective = True

        if self.timestamp is None:
            self.timestamp = time()

        return ret

    @abstractmethod
    def next_solution(self):
        """Return another solution for the effective designator or None if none exists. The next solution is a newly
        constructed designator with identical properties that is equated to the designator since it describes the same
        entity. """
        pass

    def solutions(self, from_root: Optional[Designator] = None):
        """Return a generator for all solutions of the designator.

        Arguments:
        from_root -- if not None, the generator for all solutions beginning from with the original designator is returned (default is None).
        """
        if from_root is not None:
            desig = self.first()
        else:
            desig = self

        def generator(desig):
            while desig is not None:
                try:
                    yield desig.reference()
                except DesignatorError:
                    pass

                desig = desig.next_solution()

        return generator(desig)

    def copy(self, new_properties: Optional[List] = None) -> Designator:
        """Construct a new designator with the same properties as this one. If new properties are specified, these will
        be merged with the old ones while the new properties are dominant in this relation.

        Arguments:
        new_properties -- a list of new properties to merge into the old ones (default is None).
        """
        description = self._description.copy()

        if new_properties:
            for key, value in new_properties:
                description.__dict__[key] = value

        return self.__class__(description)

    def make_effective(self, properties: Optional[List] = None,
                       data: Optional[Any] = None,
                       timestamp: Optional[float] = None) -> Designator:
        """Create a new effective designator of the same type as this one. If no properties are specified, this ones are used.

        Arguments:
        new_properties -- a list of properties (default is None).
        data -- the low-level data structure the new designator describes (default is None).
        timestamp -- the timestamp of creation of reference (default is the current).
        """
        if properties is None:
            properties = self._description

        desig = self.__class__(properties)
        desig._effective = True
        desig._data = data

        if timestamp is None:
            desig.timestamp = time()
        else:
            desig.timestamp = timestamp

        return desig

    def newest_effective(self) -> Designator:
        """Return the newest effective designator."""

        def find_effective(desig):
            if desig is None or desig._effective:
                return desig

            return find_effective(desig._parent)

        return find_effective(self.current())

    def prop_value(self, key: str) -> Any:
        """Return the first value matching the specified property key.

        Arguments:
        key -- the key to return the value of.
        """
        try:
            return self._description.__dict__[key]
        except KeyError:
            logging.error(f"The given key '{key}' is not in this Designator")
            return None

    def check_constraints(self, properties: List) -> bool:
        """Return True if all the given properties match, False otherwise.

        Arguments:
        properties -- the properties which have to match. A property can be a tuple in which case its first value is the
        key of a property which must equal the second value. Otherwise it's simply the key of a property which must be
        not None.
        """
        for prop in properties:
            if type(prop) == tuple:
                key, value = prop

                if self.prop_value(key) != value:
                    return False
            else:
                if self.prop_value(prop) is None:
                    return False

        return True

    def make_dictionary(self, properties: List) -> Dict:
        """ DEPRECATED, Moved to the description. Function only keept because of
        backward compatability.
        Return the given properties as dictionary.

        Arguments:
        properties -- the properties to create a dictionary of. A property can be a tuple in which case its first value
        is the dictionary key and the second value is the dictionary value. Otherwise it's simply the dictionary key
        and the key of a property which is the dictionary value.
        """

        return self._description.make_dictionary(properties)

    def rename_prop(self, old: str, new: str) -> Designator:
        old_value = self.prop_value(old)
        if old_value is not None:
            self._description.__dict__[new] = old_value
            del self._description.__dict__[old]
        else:
            raise DesignatorError("Old property does not exists.")
        return self.current()


class DesignatorDescription(ABC):
    """
    :ivar resolve: The specialized_designators function to use for this designator, defaults to self.ground
    """

    def __init__(self, resolver: Optional[Callable] = None, ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Create a Designator description.

        :param resolver: The grounding method used for the description. The grounding method creates a location instance that matches the description.
        :param ontology_concept_holders: A list of holders of ontology concepts that the designator is categorized as or associated with
        """

        if resolver is None:
            self.resolve = self.ground
        self.ontology_concept_holders = [] if ontology_concept_holders is None else ontology_concept_holders

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
        Returns a list of all slots of this description. Can be used for inspecting different descriptions and debugging.

        :return: A list of all slots.
        """
        return list(self.__dict__.keys())

    def copy(self) -> DesignatorDescription:
        return self

    def get_default_ontology_concept(self) -> owlready2.Thing | None:
        """
        Returns the first element of ontology_concept_holders if there is, else None
        """
        return self.ontology_concept_holders[0].ontology_concept if self.ontology_concept_holders else None

class ActionDesignatorDescription(DesignatorDescription, Language):
    """
    Abstract class for action designator descriptions.
    Descriptions hold possible parameter ranges for action designators.
    """

    @dataclass
    class Action:
        """
        The performable designator with a single element for each list of possible parameter.
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
            self.robot_torso_height = World.robot.get_joint_position(RobotDescription.current_robot_description.torso_joint)
            self.robot_type = World.robot.obj_type

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

    def __init__(self, resolver=None, ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Base of all action designator descriptions.

        :param resolver: An alternative resolver that returns an action designator
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        Language.__init__(self)
        from .ontology.ontology import OntologyManager
        self.soma = OntologyManager().soma

    def ground(self) -> Action:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")

    def init_ontology_concepts(self, ontology_concept_classes: Dict[str, Type[owlready2.Thing]]):
        """
        Initialize the ontology concept holders for this action designator

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

    def __iter__(self):
        """
        Iterate through all possible performables fitting this description

        :yield: A resolved action designator
        """
        yield self.ground()


class LocationDesignatorDescription(DesignatorDescription):
    """
    Parent class of location designator descriptions.
    """

    @dataclass
    class Location:
        """
        Resolved location that represents a specific point in the world which satisfies the constraints of the location
        designator description.
        """
        pose: Pose
        """
        The resolved pose of the location designator. Pose is inherited by all location designator.
        """

    def __init__(self, resolver=None, ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        super().__init__(resolver, ontology_concept_holders)

    def ground(self) -> Location:
        """
        Find a location that satisfies all constrains.
        """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


#this knowledge should be somewhere else i guess
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
    Class for object designator descriptions.
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
            return ORMObjectDesignator(self.obj_type, self.name)

        def insert(self, session: Session) -> ORMObjectDesignator:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :return: The completely instanced ORM object
            """
            metadata = ProcessMetaData().insert(session)
            pose = self.pose.insert(session)

            # create object orm designator
            obj = self.to_sql()
            obj.process_metadata = metadata
            obj.pose = pose
            session.add(obj)
            return obj

        def frozen_copy(self) -> 'ObjectDesignatorDescription.Object':
            """
            Returns a copy of this designator containing only the fields.

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

        def special_knowledge_adjustment_pose(self, grasp: str, pose: Pose) -> Pose:
            """
            Returns the adjusted target pose based on special knowledge for "grasp front".

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
                    rospy.loginfo("Adjusted target pose based on special knowledge for grasp: %s", grasp)
                    return pose_in_object
            return pose

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[ObjectType]] = None,
                 resolver: Optional[Callable] = None, ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        """
        Base of all object designator descriptions. Every object designator has the name and type of the object.

        :param names: A list of names that could describe the object
        :param types: A list of types that could represent the object
        :param resolver: An alternative specialized_designators that returns an object designator for the list of names and types
        :param ontology_concept_holders: A list of ontology concepts that the object is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.types: Optional[List[ObjectType]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> Union[Object, bool]:
        """
        Return the first object from the world that fits the description.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Object]:
        """
        Iterate through all possible objects fitting this description

        :yield: A resolved object designator
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