# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta
from typing import get_type_hints

from entity_query_language import an, entity, let, contains, symbolic_mode
from semantic_world.robots import AbstractRobot
from semantic_world.world_description.world_entity import Body
from typing_extensions import Type, List, Dict, Any, Optional, Callable, Self, Iterator, Iterable, Union

from .datastructures.enums import ObjectType
from .datastructures.partial_designator import PartialDesignator
from .datastructures.pose import PoseStamped

from .plan import Plan, PlanNode
from .utils import bcolors

from semantic_world.world import World


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

    plan_node: PlanNode = None

    @property
    def plan(self) -> Plan:
        """
        Returns the plan that this designator_description is part of.
        """
        if self.plan_node is not None:
            return self.plan_node.plan
        else:
            raise ValueError("This designator_description is not part of a plan.")

    @property
    def robot_view(self) -> AbstractRobot:
        """
        Returns the robot that this designator_description is part of.
        """
        if self.plan_node is not None:
            return self.plan.robot
        else:
            raise ValueError("This designator_description is not part of a plan.")

    @property
    def world(self) -> World:
        """
        Returns the world that this designator_description is part of.
        """
        if self.plan_node is not None:
            return self.plan_node.plan.world
        else:
            raise ValueError("This designator_description is not part of a plan.")

    def __init__(self):
        """
        Create a Designator description.
        """
        pass

    def resolve(self):
        return self.ground()

    def ground(self) -> Any:
        """
        Should be overwritten with an actual grounding function which infers missing properties.
        """
        return self


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

    def ground(self) -> Body:
        """
        Return the first object from the world that fits the description.

        :return: A executed object designator_description
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[Body]:
        """
        Iterate through all possible objects fitting this description

        :yield: A executed object designator_description
        """
        for params in self.generate_permutations():

            # for every world object
            for obj in self.world.bodies:

                # skip if name does not match specification
                if self.names and obj.name not in params.values():
                    continue

                # yield self.Object(obj.name, obj.obj_type, obj)
                yield obj

    def flatten(self) -> List:
        res = [None] * 7
        res.append(self.types[0])
        return res

class EQLObjectDesignator(DesignatorDescription):
    """
    Description for objects found via an EQL query.
    """
    def __init__(self, eql_query):
        super().__init__()
        self.eql_query = eql_query

    def __iter__(self) -> Iterator[Body]:
        for obj in self.eql_query.evaluate():
            yield obj

class NamedObject(ObjectDesignatorDescription, PartialDesignator):
    """
    Description for objects with a specific name.
    """

    def __init__(self, name: Union[Iterable[str], str]):
        """
        Create a description for an object with a specific name.

        :param name: The name of the object.
        """
        super().__init__()
        PartialDesignator.__init__(self, ObjectDesignatorDescription, names=name)

    def __iter__(self) -> Iterator[Body]:
        """
        Iterate through all possible objects fitting this description

        :yield: A executed object designator_description
        """
        for params in self.generate_permutations():
            with symbolic_mode():
                query = an(entity(body := let(type_=Body, domain=self.world.bodies),
                                  contains(body.name.name, params['names'])))

            for obj in query.evaluate():
                yield obj
