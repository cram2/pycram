import inspect

import rospy

from .knowledge_source import KnowledgeSource
from ..designator import DesignatorDescription, ActionDesignatorDescription
from typing_extensions import Type, Callable, List

from ..plan_failures import KnowledgeNotAvailable


class KnowledgeEngine:
    """
    The knowledge engine is responsible for managing the knowledge sources and querying them to fill parameters of
    designators
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(KnowledgeEngine, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self.knowledge_sources = []
        # Initialize all knowledge sources
        self.knowledge_sources: List[KnowledgeSource] = []
        self.init_sources()
        self._initialized: bool = True

    def init_sources(self):
        """
        Initialize all knowledge sources
        """
        # Class reference to all knowledge sources
        sources = KnowledgeSource.__subclasses__()
        for src in sources:
            self.knowledge_sources.append(src())
        self.knowledge_sources.sort(key=lambda x: x.priority)

    def update_sources(self):
        """
        Update all knowledge sources, this will check if the sources are still and if new sources have become available.
        """
        for source in self.knowledge_sources:
            if source.is_connected and not source.is_available:
                rospy.logwarn(f"Knowledge source {source.name} is not available anymore")
            elif not source.is_connected and source.is_available:
                source.connect()

    def query(self, designator: Type[ActionDesignatorDescription]):
        """
        Query to fill parameters of a designator from the knowledge sources

        :return:
        """
        conditions = designator.knowledge_conditions
        conditions(designator)

    def update(self):
        """
        Update the knowledge sources with new information contained in a designator

        :return:
        """
        ...

    def ground_solution(self, designator: Type[DesignatorDescription]) -> bool:
        """
        Try to ground a solution from the knowledge sources in the belief state

        :return: True if the solution achieves the desired goal, False otherwise
        """
        ...

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
            print(source)
            if (query_function.__name__ in list(source.__class__.__dict__.keys())
                    and source.is_connected):
                source_query_function = getattr(source, query_function.__name__)
                return source_query_function(*args, **kwargs)
        raise KnowledgeNotAvailable(f"Query function {query_function.__name__} is not available in any connected knowledge source")

