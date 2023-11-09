# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import dataclasses
from abc import ABC, abstractmethod
from copy import copy
from inspect import isgenerator, isgeneratorfunction

import rospy
import sqlalchemy

from .bullet_world import (Object as BulletWorldObject, BulletWorld)
from .helper import GeneratorList, bcolors
from threading import Lock
from time import time
from typing import List, Dict, Any, Type, Optional, Union, get_type_hints, Callable, Tuple, Iterable

from .local_transformer import LocalTransformer
from .pose import Pose
from .robot_descriptions import robot_description

import logging

from .orm.action_designator import (Action as ORMAction)
from .orm.object_designator import (ObjectDesignator as ORMObjectDesignator)

from .orm.base import Quaternion, Position, Base, RobotState, MetaData
from .task import with_tree


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
    parameters for executing actions from these pairs of key and value.

    :ivar timestamp: The timestamp of creation of reference or None if still not referencing an object.
    """


    resolvers = {}
    """
    List of all designator resolvers. Designator resolvers are functions which take a designator as
    argument and return a list of solutions. A solution can also be a generator. 
    """

    def __init__(self, description: Type[DesignatorDescription], parent: Optional[Designator] = None):
        """Create a new desginator.

        Arguments:
        :param properties: A list of tuples (key-value pairs) describing this designator.
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
        self._description: Type[DesignatorDescription] = description

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
    :ivar resolve: The resolver function to use for this designator, defaults to self.ground
    """

    def __init__(self, resolver: Optional[Callable] = None):
        """
        Create a Designator description.

        :param resolver: The grounding method used for the description. The grounding method creates a location instance that matches the description.
        """

        if resolver is None:
            self.resolve = self.ground

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

    def copy(self) -> Type[DesignatorDescription]:
        return self


class MotionDesignatorDescription(DesignatorDescription):
    """
    Parent class of motion designator descriptions.
    """

    @dataclasses.dataclass
    class Motion:
        """
        Resolved motion designator which can be performed
        """
        cmd: str
        """
        Command of this motion designator, is used to match process modules to motion designator. Cmd is inherited by 
        every motion designator.
        """

        def perform(self):
            """
            Passes this designator to the process module for execution.

            :return: The return value of the process module if there is any.
            """
            raise NotImplementedError()
            # return ProcessModule.perform(self)

    def ground(self) -> Motion:
        """Fill all missing parameters and pass the designator to the process module. """
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

    def __init__(self, resolver=None):
        """
        Creates a new motion designator description

        :param resolver: An alternative resolver which overrides self.resolve()
        """
        super().__init__(resolver)

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
        robot_position: Pose = dataclasses.field(init=False)
        """
        The position of the robot at the start of the action.
        """
        robot_torso_height: float = dataclasses.field(init=False)
        """
        The torso height of the robot at the start of the action.
        """

        robot_type: str = dataclasses.field(init=False)
        """
        The type of the robot at the start of the action.
        """

        def __post_init__(self):
            self.robot_position = BulletWorld.robot.get_pose()
            self.robot_torso_height = BulletWorld.robot.get_joint_state(robot_description.torso_joint)
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

            # get or create metadata
            metadata = MetaData().insert(session)

            # create position
            position = Position(*self.robot_position.position_as_list())
            position.metadata_id = metadata.id

            # create orientation
            orientation = Quaternion(*self.robot_position.orientation_as_list())
            orientation.metadata_id = metadata.id

            session.add_all([position, orientation])
            session.commit()

            # create robot-state object
            robot_state = RobotState()
            robot_state.position = position.id
            robot_state.orientation = orientation.id
            robot_state.torso_height = self.robot_torso_height
            robot_state.type = self.robot_type
            robot_state.metadata_id = metadata.id
            session.add(robot_state)
            session.commit()

            # create action
            action = self.to_sql()
            action.metadata_id = metadata.id
            action.robot_state = robot_state.id

            return action

    def __init__(self, resolver=None):
        super(ActionDesignatorDescription, self).__init__(resolver)

    def ground(self) -> Action:
        """Fill all missing parameters and chose plan to execute. """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


class LocationDesignatorDescription(DesignatorDescription):
    """
    Parent class of location designator descriptions.
    """

    @dataclasses.dataclass
    class Location:
        """
        Resolved location that represents a specific point in the world which satisfies the constraints of the location
        designator description.
        """
        pose: Pose
        """
        The resolved pose of the location designator. Pose is inherited by all location designator.
        """

    def __init__(self, resolver=None):
        super().__init__(resolver)

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

    @dataclasses.dataclass
    class Object:
        """
        A single element that fits the description.
        """

        name: str
        """
        Name of the object
        """

        type: str
        """
        Type of the object
        """

        bullet_world_object: Optional[BulletWorldObject]
        """
        Reference to the BulletWorld object
        """

        _pose: Optional[Callable] = dataclasses.field(init=False)
        """
        A callable returning the pose of this object. The _pose member is used overwritten for data copies
        which will not update when the original bullet_world_object is moved.
        """

        def __post_init__(self):
            if self.bullet_world_object:
                self._pose = self.bullet_world_object.get_pose

        def to_sql(self) -> ORMObjectDesignator:
            """
            Create an ORM object that corresponds to this description.

            :return: The created ORM object.
            """
            return ORMObjectDesignator(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectDesignator:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :return: The completely instanced ORM object
            """
            metadata = MetaData().insert(session)
            # insert position and orientation of object of the designator
            orm_position = Position(*self.pose.position_as_list(), metadata.id)
            orm_orientation = Quaternion(*self.pose.orientation_as_list(), metadata.id)
            session.add(orm_position)
            session.add(orm_orientation)
            session.commit()

            # create object orm designator
            obj = self.to_sql()
            obj.metadata_id = metadata.id
            obj.position = orm_position.id
            obj.orientation = orm_orientation.id
            session.add(obj)
            session.commit()
            return obj

        def data_copy(self) -> 'ObjectDesignatorDescription.Object':
            """
            :return: A copy containing only the fields of this class. The BulletWorldObject attached to this pycram
            object is not copied. The _pose gets set to a method that statically returns the pose of the object when
            this method was called.
            """
            result = ObjectDesignatorDescription.Object(self.name, self.type, None)
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
                [f"{f.name}={self.__getattribute__(f.name)}" for f in dataclasses.fields(self)] + [
                    f"pose={self.pose}"]) + ')'

        def special_knowledge_adjustment_pose(self, grasp: str, pose: Pose) -> Pose:
            """
            Returns the adjusted target pose based on special knowledge for "grasp front".

            :param grasp: From which side the object should be grasped
            :param pose: Pose at which the object should be grasped, before adjustment
            :return: The adjusted grasp pose
            """
            lt = LocalTransformer()
            pose_in_object = lt.transform_to_object_frame(pose, self.bullet_world_object)

            special_knowledge = []  # Initialize as an empty list
            if self.type in SPECIAL_KNOWLEDGE:
                special_knowledge = SPECIAL_KNOWLEDGE[self.type]

            for key, value in special_knowledge:
                if key == grasp:
                    # Adjust target pose based on special knowledge
                    pose_in_object.pose.position.x += value[0]
                    pose_in_object.pose.position.y += value[1]
                    pose_in_object.pose.position.z += value[2]
                    rospy.loginfo("Adjusted target pose based on special knowledge for grasp: " + grasp)
                    return pose_in_object
            return pose

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[str]] = None,
                 resolver: Optional[Callable] = None):
        """
        Base of all object designator descriptions. Every object designator has the name and type of the object.

        :param names: A list of names that could describe the object
        :param types: A list of types that could represent the object
        :param resolver: An alternative resolver that returns an object designator for the list of names and types
        """
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> Union[Object, bool]:
        """
        Return the first object from the bullet world that fits the description.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Object]:
        """
        Iterate through all possible objects fitting this description

        :yield: A resolved object designator
        """
        # for every bullet world object
        for obj in BulletWorld.current_bullet_world.objects:

            # skip if name does not match specification
            if self.names and obj.name not in self.names:
                continue

            # skip if type does not match specification
            if self.types and obj.type not in self.types:
                continue

            yield self.Object(obj.name, obj.type, obj)