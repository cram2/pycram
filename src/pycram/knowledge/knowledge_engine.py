from __future__ import annotations

import inspect

import rospy
from anytree import PreOrderIter
from typeguard import check_type, TypeCheckError

from ..datastructures.property import Property, ResolvedProperty
from .knowledge_source import KnowledgeSource
# from ..designator import DesignatorDescription, ActionDesignatorDescription
from typing_extensions import Type, Callable, List, TYPE_CHECKING, Dict

from ..plan_failures import KnowledgeNotAvailable
# This import is needed since the subclasses of KnowledgeSource need to be imported to be known at runtime
from .knowledge_sources import *
if TYPE_CHECKING:
    from ..designator import ActionDesignatorDescription


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
        if self._initialized: return
        if not self.enabled:
            rospy.logwarn("Knowledge engine is disabled")
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
                rospy.logwarn(f"Knowledge source {source.name} is not available anymore")
            elif not source.is_connected and source.is_available:
                source.connect()

    def query(self, designator: Type['ActionDesignatorDescription']) -> bool:
        """
        Query to fill parameters of a designator from the knowledge sources

        :return:
        """
        if not self.enabled:
            rospy.logwarn("Knowledge engine is disabled")
            return True
        self.update_sources()

        condition = self.resolve_aspects(designator.knowledge_condition)
        return condition(designator)

    def resolve_aspects(self, aspects: Property):
        """
        Traverses the tree of properties and resolves the property functions to the corresponding function in the knowledge
        source.

        :param aspects: Root node of the tree of properties
        """
        for child in PreOrderIter(aspects):
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
        return node.root

    def update(self):
        """
        Update the knowledge sources with new information contained in a designator

        :return:
        """
        if not self.enabled:
            rospy.logwarn("Knowledge engine is disabled")
            return True

    def ground_solution(self, designator: Type['DesignatorDescription']) -> bool:
        """
        Try to ground a solution from the knowledge sources in the belief state

        :return: True if the solution achieves the desired goal, False otherwise
        """
        if not  self.enabled:
            rospy.logwarn("Knowledge engine is disabled")
            return True

    def call_source(self, query_function: Callable, *args, **kwargs):
        """
        Call the given query function on the knowledge source with the highest priority that is connected

        :param query_function: The query function of the knowledge source
        :param args: The arguments to pass to the query function
        :param kwargs: The keyword arguments to pass to the query function
        :return: The result of the query function
        """
        self.update_sources()
        for source in self.knowledge_sources:
            if (query_function.__name__ in list(source.__class__.__dict__.keys())
                    and source.is_connected):
                source_query_function = getattr(source, query_function.__name__)
                return source_query_function(*args, **kwargs)
        raise KnowledgeNotAvailable(
            f"Query function {query_function.__name__} is not available in any connected knowledge source")

    def find_source_for_property(self, property: Type[Property]):
        """
        Find the source for the given property

        :param property: The property for which to find the source.
        :return: Source that can provide the property.
        """
        for source in self.knowledge_sources:
            if (property.__class__ in list(source.__class__.__bases__)
                    and source.is_connected):
                return source

    def match_reasoned_parameter(self, condition: Type[Property], designator: Type[ActionDesignatorDescription]):
        """
        Match the reasoned parameters, in the root node of the property expression, to the corresponding parameter in
        the designator
        """
        parameter = condition.root.variables
        self._match_by_name(parameter, designator)
        self._match_by_type(parameter, designator)

    @staticmethod
    def _match_by_name(parameter: Dict[str, any], designator: Type[ActionDesignatorDescription]):
        """
        Match the reasoned parameters to the corresponding parameter in the designator by name
        """
        for key, value in parameter.items():
            if key in designator.get_optional_parameter() and designator.__getattribute__(key) is None:
                designator.__setattr__(key, value)

    @staticmethod
    def _match_by_type(parameter: Dict[str, any], designator: Type[ActionDesignatorDescription]):
        """
        Match the reasoned parameters to the corresponding parameter in the designator by type
        """
        for param in designator.get_optional_parameter():
            if designator.__getattribute__(param) is None:
                for key, value in parameter.items():
                    try:
                        check_type(value, designator.get_type_hints()[param])
                        designator.__setattr__(param, value)
                    except TypeCheckError as e:
                        continue
