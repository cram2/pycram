# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta
from typing import get_type_hints

from typing_extensions import Type, List, Dict, Any, Optional, Callable, Self, Iterator

from pycrap.ontologies import PhysicalObject, Agent
from .datastructures.enums import ObjectType
from .datastructures.partial_designator import PartialDesignator
from .datastructures.pose import PoseStamped
from .datastructures.world import World
from .failures import PlanFailure
from .has_parameters import HasParameters
from .robot_description import RobotDescription
from .utils import bcolors
from .world_concepts.world_object import Object as WorldObject, Object


class DesignatorError(Exception):
    """Implementation of designator_description errors."""

    def __init__(self, *args, **kwargs):
        """Create a new designator_description error."""
        Exception.__init__(self, *args, **kwargs)


class ResolutionError(Exception):
    def __init__(self, missing_properties: List[str], wrong_type: Dict, current_type: Any,
                 designator: DesignatorDescription):
        self.error = (f"\nSome required properties where missing or had the wrong type when grounding the Designator:"
                      f" {designator}.\n")
        self.missing = f"The missing properties where: {missing_properties}\n"
        self.wrong = f"The properties with the wrong type along with the current -and right type :\n"
        self.head = ("Property   |   Current Type    |     Right Type\n------------------------------------"
                     "-------------------------\n")
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


class DesignatorDescription:
    """
    :ivar resolve: The specialized_designators function to use for this designator_description, defaults to self.ground
    """

    def __init__(self):
        """
        Create a Designator description.
        """
        pass

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

    @classmethod
    def get_type_hints(cls) -> Dict[str, Any]:
        """
        Returns the type hints of the __init__ method of this designator_description description.

        :return:
        """
        return get_type_hints(cls.__init__)




class LocationDesignatorDescription(DesignatorDescription, PartialDesignator):
    """
    Parent class of location designator_description descriptions.
    """

    def __init__(self):
        super().__init__()

    def ground(self) -> PoseStamped:
        """
        Find a location that satisfies all constrains.
        """
        raise NotImplementedError(f"{type(self)}.ground() is not implemented.")


class ObjectDesignatorDescription(DesignatorDescription, PartialDesignator):
    """
    Class for object designator_description descriptions.
    Descriptions hold possible parameter ranges for object designators.
    """

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[Type[PhysicalObject]]] = None):
        """
        Base of all object designator_description descriptions. Every object designator_description has the name and type of the object.

        :param names: A list of names that could describe the object
        :param types: A list of types that could represent the object
        """
        super().__init__()
        PartialDesignator.__init__(self, ObjectDesignatorDescription, names=names, types=types)
        self.types: Optional[List[ObjectType]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> WorldObject:
        """
        Return the first object from the world that fits the description.

        :return: A executed object designator_description
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[WorldObject]:
        """
        Iterate through all possible objects fitting this description

        :yield: A executed object designator_description
        """
        for params in self.generate_permutations():

            # for every world object
            for obj in World.current_world.objects:

                # skip if name does not match specification
                if self.names and obj.name not in params.values():
                    continue

                # skip if type does not match specification
                if self.types and obj.obj_type not in params.values():
                    continue

                # yield self.Object(obj.name, obj.obj_type, obj)
                yield obj

    def flatten(self) -> List:
        res = [None] * 7
        res.append(self.types[0])
        return res


@dataclass
class BaseMotion(ABC):

    @abstractmethod
    def perform(self):
        """
        Passes this designator to the process module for execution. Will be overwritten by each motion.
        """
        pass

    def __post_init__(self):
        """
        Checks if types are missing or wrong
        """
        return
        # TODO include type checks for this again (use type guard?)
        #
        # right_types = get_type_hints(self)
        # attributes = self.__dict__.copy()
        #
        # missing = []
        # wrong_type = {}
        # current_type = {}
        #
        # for k in attributes.keys():
        #     attribute = attributes[k]
        #     attribute_type = type(attributes[k])
        #     right_type = right_types[k]
        #     types = get_args(right_type)
        #     if attribute is None:
        #         if not any([x is type(None) for x in get_args(right_type)]):
        #             missing.append(k)
        #     elif not issubclass(attribute_type, right_type): # attribute_type is not right_type:
        #         if attribute_type not in types:
        #             if attribute_type not in [get_origin(x) for x in types if x is not type(None)]:
        #                 wrong_type[k] = right_types[k]
        #                 current_type[k] = attribute_type
        # if missing != [] or wrong_type != {}:
        #     raise ResolutionError(missing, wrong_type, current_type, self.__class__)
        #
