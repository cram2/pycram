from __future__ import annotations

import inspect
from enum import Enum
from typing import _GenericAlias

from anytree import PreOrderIter
from typeguard import check_type, TypeCheckError

from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.property import Property, ResolvedProperty, PropertyOperator
from .knowledge_source import KnowledgeSource
from typing_extensions import Type, Callable, List, TYPE_CHECKING, Dict, Any

from ..failures import KnowledgeNotAvailable, ReasoningError
from ..ros import logwarn

if TYPE_CHECKING:
    from ..designator import ActionDescription


class KnowledgeEngine:
    """
    The knowledge engine is responsible for managing the knowledge sources and querying them to fill parameters of
    designators
    """

    enabled = True
    """
    Flag to enable or disable the knowledge engine
    """

    _instance = None
    """
    Singleton for the knowledge engine
    """

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(KnowledgeEngine, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """
        Initialize the knowledge engine, this includes the collection of all knowledge sources and the initialization of
        each source.
        """
        if self._initialized: return
        if not self.enabled:
            logwarn("Knowledge engine is disabled")
            return
        self.knowledge_sources = []
        # Initialize all knowledge sources
        self.knowledge_sources: List[KnowledgeSource] = []
        self.init_sources()
        self._initialized: bool = True

    def init_sources(self):
        """
        Initialize all knowledge sources from the available subclasses of KnowledgeSource
        """
        # Class reference to all knowledge sources
        sources = KnowledgeSource.__subclasses__()
        for src in sources:
            if src not in [c.__class__ for c in self.knowledge_sources]:
                self.knowledge_sources.append(src())
        self.knowledge_sources.sort(key=lambda x: x.priority)

    def update_sources(self):
        """
        Updates all knowledge sources, this will check if the sources are still available and if new sources have
        become available.
        """
        self.init_sources()
        for source in self.knowledge_sources:
            if source.is_connected and not source.is_available:
                logwarn(f"Knowledge source {source.name} is not available anymore")
            elif not source.is_connected and source.is_available:
                source.connect()

    def query(self, designator: ActionDescription) -> bool:
        """
        Query to fill parameters of a designator_description from the knowledge sources

        :return:
        """
        if not self.enabled:
            logwarn("Knowledge engine is disabled")
            return True
        self.update_sources()

        condition = self.resolve_properties(designator.knowledge_condition)
        return condition(designator)

    def resolve_properties(self, properties: Property):
        """
        Traverses the tree of properties and resolves the property functions to the corresponding function in the knowledge
        source. Properties are executed in-place.

        :param properties: Root node of the tree of properties
        """
        if properties.resolved:
            return properties.root
        for child in PreOrderIter(properties):
            if child.is_leaf:
                source = self.find_source_for_property(child)
                resolved_aspect_function = source.__getattribute__(
                    [fun for fun in child.__class__.__dict__.keys() if
                     not fun.startswith("__") and not fun == "property_exception"][0])

                # child.resolved_property_instance = source
                node = ResolvedProperty(resolved_aspect_function, child.property_exception, child.parent)
                for param in inspect.signature(resolved_aspect_function).parameters.keys():
                    node.parameter[param] = child.__getattribute__(param)
                child.parent = None

        if isinstance(properties.root, PropertyOperator):
            return properties.root
        else:
            return node

    def update(self):
        """
        Update the knowledge sources with new information contained in a designator_description

        :return:
        """
        if not self.enabled:
            logwarn("Knowledge engine is disabled")
            return True

    def ground_solution(self, designator: ActionDescription) -> bool:
        """
        Try to ground a solution from the knowledge sources in the belief state

        :return: True if the solution achieves the desired goal, False otherwise
        """
        if not self.enabled:
            logwarn("Knowledge engine is disabled")
            return True

    def find_source_for_property(self, property: Type[Property]) -> KnowledgeSource:
        """
        Find the source for the given property

        :param property: The property for which to find the source.
        :return: Source that can provide the property.
        """
        for source in self.knowledge_sources:
            if (property.__class__ in list(source.__class__.__bases__)
                    and source.is_connected):
                return source

    def match_reasoned_parameter(self, reasoned_parameter: Dict[str, any],
                                 designator: ActionDescription) -> Dict[str, any]:
        """
        Match the reasoned parameters, in the root node of the property expression, to the corresponding parameter in
        the designator_description
        """
        matched_parameter = {}
        matched_parameter.update(self._match_by_name(reasoned_parameter, designator))
        matched_parameter.update(self._match_by_type(reasoned_parameter, designator))
        return matched_parameter

    @staticmethod
    def _match_by_name(parameter: Dict[str, any], designator: ActionDescription) -> Dict[str, any]:
        """
        Match the reasoned parameters to the corresponding parameter in the designator_description by name
        """
        result_dict = {}
        for key, value in parameter.items():
            # if key in designator_description.get_optional_parameter() and designator_description.__getattribute__(key) is None:
            if key in designator.get_type_hints(localns=locals()).keys():
                result_dict[key] = value
        return result_dict

    @staticmethod
    def _match_by_type(parameter: Dict[str, any], designator: ActionDescription) -> Dict[str, any]:
        """
        Match the reasoned parameters to the corresponding parameter in the designator_description by type
        """
        result_dict = {}
        for key, value in parameter.items():
            for parameter_name, type_hint in designator.get_type_hints(localns=locals()).items():
                try:
                    # Distinction between Enum and other types, since check_type would check Enums and floats as an Enum
                    # is technically just a number. Also excludes type hints, since they do not work with issubclass
                    if issubclass(value.__class__, Enum) and not issubclass(type_hint.__class__, _GenericAlias):
                        if not issubclass(value.__class__, type_hint):
                            raise TypeCheckError(f"Expected type {type_hint} but got {value.__class__}")
                    else:
                        check_type(value, type_hint)

                    result_dict[parameter_name] = value
                except TypeCheckError as e:
                    continue
        return result_dict

class ReasoningInstance:
    """
    A reasoning instance is a generator class that reasons about the missing parameter as well as the feasibility of
    the designator_description given.
    Since this is a generator it keeps its state while yielding full designator and can be used to generate the next
    full designator at a later time.
    """

    def __init__(self, designator_description: ActionDescription, partial_designator: PartialDesignator):
        """
        Initialize the reasoning instance with the designator_description and the partial designator

        :param designator_description: The description from which the designator should be created
        :param partial_designator: A partial initialized designator_description using the PartialDesignator class
        """
        self.designator_description = designator_description
        self.knowledge_engine = KnowledgeEngine()
        self.resolved_property = self.knowledge_engine.resolve_properties(self.designator_description.knowledge_condition)
        self.parameter_to_be_reasoned = [param_name for param_name in self.designator_description.get_optional_parameter() if
                                         self.designator_description.__getattribute__(param_name) is None]
        self.partial_designator = partial_designator

    def __iter__(self) -> ActionDescription:
        """
        Executes property structure, matches the reasoned and missing parameter and generates a completes designator.

        :yield: A complete designator with all parameters filled
        """
        self.resolved_property(self.designator_description)

        matched_parameter = self.knowledge_engine.match_reasoned_parameter(self.resolved_property.root.variables, self.designator_description)
        # Check if the parameter that need to be filled are contained in the set of reasoned parameters
        if not set(matched_parameter).issuperset(set(self.partial_designator.missing_parameter())):
            not_reasoned = set(self.partial_designator.missing_parameter()).difference(set(matched_parameter))
            raise ReasoningError(f"The parameters {not_reasoned} could not be inferred from the knowledge sources. Therefore, a complete designator can not be generated. ")

        for key, value in self.partial_designator.kwargs.items():
            # This line means the parameter of the designator description will be preferred over the reasoned parameter
            if value is None:
                self.partial_designator.kwargs[key] = matched_parameter[key]


        for designator in self.partial_designator(**matched_parameter):
            yield designator
