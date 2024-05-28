from .knowledge_source import KnowledgeSource
from ..designator import DesignatorDescription
from typing_extensions import Type


class KnowledgeEngine:
    """
    The knowledge engine is responsible for managing the knowledge sources and querying them to fill parameters of
    designators
    """
    def __init__(self):
        self.knowledge_sources = []
        # Initialize all knowledge sources
        sources = KnowledgeSource.__subclasses__()
        for src in sources:
            self.knowledge_sources.append(src())
        self.knowledge_sources.sort(key=lambda x: x.priority)

    def query(self, designator: Type[DesignatorDescription]):
        """
        Query to fill parameters of a designator from the knowledge sources

        :return:
        """
        ...

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

