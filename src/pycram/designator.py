"""Implementation of designators.

Classes:
DesignatorError -- implementation of designator errors.
Designator -- implementation of designators.
MotionDesignator -- implementation of motion designators.
"""
# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from abc import ABC, abstractmethod
from copy import copy
from inspect import isgenerator, isgeneratorfunction

from .helper import GeneratorList, bcolors
from threading import Lock
from time import time
from typing import List, Dict, Any, Type, Optional, Union, get_type_hints, Callable

import logging


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
    """Implementation of designators.

    Designators are objects containing sequences of key-value pairs. They can be resolved which means to generate real parameters for executing actions from these pairs of key and value.

    Instance variables:
    timestamp -- the timestamp of creation of reference or None if still not referencing an object.

    Methods:
    equate -- equate the designator with the given parent.
    equal -- check if the designator describes the same entity as another designator.
    first -- return the first ancestor in the chain of equated designators.
    current -- return the newest designator.
    reference -- try to dereference the designator and return its data object
    next_solution -- return another solution for the effective designator or None if none exists.
    solutions -- return a generator for all solutions of the designator.
    copy -- construct a new designator with the same properties as this one.
    make_effective -- create a new effective designator of the same type as this one.
    newest_effective -- return the newest effective designator.
    prop_value -- return the first value matching the specified property key.
    check_constraints -- return True if all the given properties match, False otherwise.
    make_dictionary -- return the given parameters as dictionary.
    """

    """List of all designator resolvers. Designator resolvers are functions which take a designator as
    argument and return a list of solutions. A solution can also be a generator. """
    resolvers = {}

    def __init__(self, description: Type[DesignatorDescription], parent: Optional[Designator] = None):
        """Create a new desginator.

        Arguments:
        properties -- a list of tuples (key-value pairs) describing this designator.
        parent -- the parent to equate with (default is None).
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
        """Return the newest designator, i.e. that one that has been equated last to the designator or one of its equated designators."""
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
        """Construct a new designator with the same properties as this one. If new properties are specified, these will be merged with the old ones while the new properties are dominant in this relation.

        Arguments:
        new_properties -- a list of new properties to merge into the old ones (default is None).
        """
        description = self._description.copy()

        if new_properties:
            for key, value in new_properties:
                description.__dict__[key] = value
                #if key in description.__dict__.keys():
                    #description.__dict__[key] = value


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
        properties -- the properties which have to match. A property can be a tuple in which case its first value is the key of a property which must equal the second value. Otherwise it's simply the key of a property which must be not None.
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
        properties -- the properties to create a dictionary of. A property can be a tuple in which case its first value is the dictionary key and the second value is the dictionary value. Otherwise it's simply the dictionary key and the key of a property which is the dictionary value.
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

    def __init__(self, grounding_method: Optional[Callable] = None):
        """
        Create a Designator description.

        :param grounding_method: The grounding method used for the description.
        The grounding method creates an action instance that matches the description.
        """

        if grounding_method is None:
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


