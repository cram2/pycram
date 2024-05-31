from ..datastructures.knowledge_source import KnowledgeSource, QueryKnowledge, UpdateKnowledge
from ..designator import DesignatorDescription


class FactsKnowledge(KnowledgeSource, QueryKnowledge, UpdateKnowledge):
    """
    Knowledge source for hard coded facts, this knowledge source acts as a fallback if no other knowledge source is
    available.
    """
    def __init__(self):
        super().__init__(name="Facts", priority=99)

    @property
    def is_available(self) -> bool:
        return True

    @property
    def is_connected(self) -> bool:
        return True

    def connect(self):
        pass

    def query(self, designator: DesignatorDescription) -> DesignatorDescription:
        """
        Query the knowledge source for the facts
        """
        pass

    def query_pose_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        pass

    def query_grasp_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        pass
