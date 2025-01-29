from collections import UserDict
from dataclasses import dataclass
from inspect import signature

from typing_extensions import Type, Optional, Self, Any, List, Dict

from pycrap.ontologies import Base


class HasConcept:
    """
    A mixin class that adds an ontological concept and individual to a class that will be registered in PyCRAP.
    """

    ontology_concept: Type[Base] = Base
    """
    The ontological concept that this class represents.
    """

    ontology_individual: Optional[Base] = None
    """
    The individual in the ontology that is connected with this class.
    """

    def __init__(self):
        self.ontology_individual = self.ontology_concept()


@dataclass
class ParameterInfo:
    """
    Class for information about a possible parameterized field.
    """
    name: str
    dtype: type
    value: Optional[Any] = None


class HasParametrization:
    """
    A mixin class that adds parameterization functionality to a class.
    """

    @classmethod
    def from_parameters(cls, parameters: Dict[str, Any]) -> Self:
        raise NotImplementedError

    @classmethod
    def parameters(cls) -> Dict[str, Any]:
        raise NotImplementedError

