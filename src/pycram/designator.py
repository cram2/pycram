# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, fields
from datetime import timedelta

from pycrap.ontologies import PhysicalObject, Agent
from .datastructures.enums import ObjectType
from .datastructures.pose import PoseStamped, GraspDescription
from .failures import PlanFailure

from .datastructures.world import World
from .datastructures.partial_designator import PartialDesignator
from typing_extensions import Type, List, Dict, Any, Optional, Union, Callable, Iterable, TYPE_CHECKING, ForwardRef, \
    Self, Iterator
from typing import get_type_hints

from .has_parameters import HasParameters
from .language import LanguageMixin
from .local_transformer import LocalTransformer
from .robot_description import RobotDescription
from .ros import loginfo
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


class DesignatorDescription(ABC):
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

    def get_type_hints(self) -> Dict[str, Any]:
        """
        Returns the type hints of the __init__ method of this designator_description description.

        :return:
        """
        return get_type_hints(self.__init__)

@dataclass
class ActionDescription():
    """
    The performable designator_description with a single element for each list of possible parameter.
    """
    robot_position: PoseStamped = field(init=False)
    """
    The position of the robot at the start of the action.
    """
    robot_torso_height: float = field(init=False)
    """
    The torso height of the robot at the start of the action.
    """

    _robot_type: Type[Agent] = field(init=False)
    """
    The type of the robot at the start of the action.
    """
    _pre_perform_callbacks = []
    """
    List of callback functions that will be called before the action is performed.
    """

    _post_perform_callbacks = []
    """
    List of callback functions that will be called after the action is performed.
    """

    def __post_init__(self):
        self.robot_position = World.robot.get_pose()
        if RobotDescription.current_robot_description.torso_joint != "":
            self.robot_torso_height = World.robot.get_joint_position(
                RobotDescription.current_robot_description.torso_joint)
        else:
            self.robot_torso_height = 0.0
        self._robot_type = World.robot.obj_type

    def perform(self) -> Any:
        """
        Executes the action with the single parameters from the description.

        :return: The result of the action in the plan
        """
        result: Optional[Any] = None
        for pre_perform in self._pre_perform_callbacks:
            pre_perform(self)
        try:
            result = self.plan()
        except PlanFailure as e:
            raise e
        finally:
            for post_perform in self._post_perform_callbacks:
                post_perform(self)
            self.validate(result)
        return result

    def plan(self) -> Any:
        """
        Plan of the action. To be overridden by subclasses.

        :return: The result of the action, if there is any
        """
        raise NotImplementedError()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Validate the action after performing it, by checking if the action effects are as expected.

        :param result: The result of the action if there is any
        :param max_wait_time: The maximum time to wait for the action to be validated, before raising an error.
        """
        raise NotImplementedError()

    @classmethod
    def get_type_hints(cls, localns=None) -> Dict[str, Any]:
        """
        Returns the type hints of the __init__ method of this designator_description description.

        :return:
        """
        l = localns if localns is not None else locals()
        return get_type_hints(cls, localns=l)

    @classmethod
    def pre_perform(cls, func) -> Callable:
        """
        Decorator to execute the decorated function before performing the action.

        :param func: The function to be decorated.
        :return: The decorated function.
        """
        cls._pre_perform_callbacks.append(func)

        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)

        return wrapper

    @classmethod
    def post_perform(cls, func) -> Callable:
        """
        Decorator to execute the decorated function after performing the action.

        :param func: The function to be decorated.
        :return: The decorated function.
        """
        cls._post_perform_callbacks.append(func)

        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)

        return wrapper

    @classmethod
    def description(cls, *args, **kwargs) -> PartialDesignator[Self]:
        raise NotImplementedError()

    def __str__(self):
        # all fields that are not ORM classes
        fields = {}
        for key, value in vars(self).items():
            if key.startswith("orm_"):
                continue
            if isinstance(value, Object):
                fields[key] = value.name
            elif isinstance(value, PoseStamped):
                fields[key] = value.__str__()
        fields_str = "\n".join([f"{key}: {value}" for key, value in fields.items()])
        return f"{self.__class__.__name__.replace('Performable', '')}:\n{fields_str}"

    def __repr__(self):
        return self.__str__()


class LocationDesignatorDescription(DesignatorDescription, PartialDesignator, Iterable[PoseStamped]):
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


class ObjectDesignatorDescription(DesignatorDescription, PartialDesignator, Iterable[WorldObject]):
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

@dataclass
class BaseMotion(ABC):

    @abstractmethod
    def perform(self):
        """
        Passes this designator to the process module for execution. Will be overwritten by each motion.
        """
        pass
        # return ProcessModule.perform(self)

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
