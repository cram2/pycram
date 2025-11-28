from __future__ import annotations

import enum
from functools import lru_cache

import typing_extensions
from typing_extensions import (
    List,
    Dict,
    Union,
    Tuple,
    TYPE_CHECKING,
    get_origin,
    get_args,
)

import logging

logger = logging.getLogger(__name__)

# Forward declaration of the class
HasParameters = object

if TYPE_CHECKING:
    LeafTypes = Union[float, int, str, bool, enum.Enum]

    ParameterDict = Dict[str, Union[LeafTypes, HasParameters]]

leaf_types = (int, float, str, bool, enum.Enum, type)


class HasParametersMeta(type):
    """
    Metaclass for flattening and reconstructing nested Python datastructures until they reach something mentioned in leaf_types.
    This is very similar to JAX PyTrees `https://docs.jax.dev/en/latest/pytrees.html#pytrees`_.
    Will scan the constructor of the class for fields to be used and not class variables.
    """

    _parameters: ParameterDict = None
    """
    A dictionary that maps field names to their types, including nested types.
    The keys are the names of the variables. The values are the types of the variables or nested types.
    """

    def __init__(cls, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        cls._parameters = {}
        cls.create_parameters(cls)

    @classmethod
    def create_parameters(cls, target_class):
        """
        Creates the flattened parameters for the given class.

        :param target_class: Class for which to create the parameters.
        """
        if not issubclass(target_class, HasParameters):
            raise TypeError(
                f"{target_class.__name__} must inherit from HasParameters. If you are using a dataclass use both the decorator has_parameters and inherit from HasParameters."
            )
        # calculate the field
        target_class._parameters = {}
        if hasattr(target_class, "define_parameters"):
            try:
                target_class._parameters = target_class.define_parameters()
                return
            except NotImplementedError:
                pass

        for field_name, field_type in typing_extensions.get_type_hints(
            target_class.__init__
        ).items():
            if field_name.startswith("_") or field_name == "return":
                continue
            # in case there are optional arguments
            if get_origin(field_type) is Union:
                type_a, type_b = get_args(field_type)
                field_type = type_a or type_b
                target_class._parameters[field_name] = field_type
                continue
            if get_origin(field_type) is type:
                type_type = get_args(field_type)[0]
                target_class._parameters[field_name] = type_type
                continue
            try:
                if issubclass(field_type, leaf_types) or issubclass(
                    field_type, HasParameters
                ):
                    target_class._parameters[field_name] = field_type
            except TypeError as e:
                logger.warning(
                    f"Filed type in {target_class.__name__} is not a leaf type: {field_type}"
                )


class HasParameters(metaclass=HasParametersMeta):
    """
    Base class for everything that contains potentially parameters for a plan.
    """

    def flatten(self) -> List:
        """
        Flattens the object into a list of field values.

        :return: A list of flattened field values from the object.

        :raises TypeError: If the object is not an instance of the target class.
        """
        result = []
        for field_name, field_type in self._parameters.items():
            if issubclass(field_type, HasParameters):
                # sub_flattener = HasParameters(field_type.clazz)
                sub_obj = getattr(self, field_name)
                result.extend(sub_obj.flatten())
            else:
                result.append(getattr(self, field_name))
        return result

    @classmethod
    @lru_cache(maxsize=None)
    def flattened_parameters(cls) -> Dict[str, leaf_types]:
        """
        Returns a dictionary of all flattened fields and their types.

        :return: A dictionary mapping field names to their types.
        """
        flat_fields = {}
        for field_name, field_type in cls._parameters.items():
            if issubclass(field_type, HasParameters):
                sub_flat_fields = field_type.flattened_parameters()
                for sub_field_name, sub_field_type in sub_flat_fields.items():
                    flat_fields[f"{field_name}.{sub_field_name}"] = sub_field_type
            elif issubclass(field_type, leaf_types):
                flat_fields[field_name] = field_type
        return flat_fields

    @classmethod
    @lru_cache(maxsize=None)
    def number_of_fields(cls) -> int:
        """
        :return: The total number of fields in the flattened list.
        """
        return len(cls.flattened_parameters())

    @classmethod
    @lru_cache(maxsize=None)
    def field_indices(cls) -> Dict[str, Tuple[int, int]]:
        """
        :return: A dictionary mapping field names to their indices in the flattened list.
        """
        field_indices = {}
        index = 0
        for field_name, field_type in cls._parameters.items():
            if issubclass(field_type, HasParameters):
                field_indices[field_name] = (
                    index,
                    index + field_type.number_of_fields(),
                )
                index += field_type.number_of_fields()
            elif issubclass(field_type, leaf_types):
                field_indices[field_name] = (index, index + 1)
                index += 1
        return field_indices

    @classmethod
    def reconstruct(cls, flattened: List):
        """
        Reconstructs the object from a flattened list of field values.

        :param flattened: The flattened list of field values.

        :return: An instance of the target class with the reconstructed field values.

        :raises TypeError: If the object is not a list or if the length of the list does not match the number of fields.
        """
        if len(flattened) != len(cls.flattened_parameters()):
            raise ValueError("List length does not match number of flattened fields")

        kwargs = {}
        for field_name, field_type in cls._parameters.items():
            start, end = cls.field_indices()[field_name]
            if issubclass(field_type, HasParameters):
                sub_obj = flattened[start:end]
                kwargs[field_name] = field_type.reconstruct(sub_obj)
            else:
                kwargs[field_name] = flattened[start]

        return cls(**kwargs)

    @classmethod
    def define_parameters(cls) -> ParameterDict:
        """
        Override this method if you want to define a custom parameter dict for this class.

        :return: Dict like cls._parameters
        """
        raise NotImplementedError


T = typing_extensions.TypeVar("T")


def has_parameters(target_class: T) -> T:
    """
    Insert parameters of a class post construction.
    Use this when dataclasses should be combined with HasParameters.

    :param target_class: The class to get the parameters from.
    :return: The updated class
    """

    def wrapped_class():
        HasParametersMeta.create_parameters(target_class)
        return target_class

    return wrapped_class()
