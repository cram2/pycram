import rospy

from ..datastructures.knowledge_source import KnowledgeSource, QueryKnowledge, UpdateKnowledge
import rosservice
from ..designator import DesignatorDescription
try:
    from rosprolog_client import Prolog
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import Prolog client from package rosprolog_client, Knowrob related features are not available.")


class KnowrobKnowledge(KnowledgeSource, QueryKnowledge, UpdateKnowledge):

    def __init__(self):
        super().__init__("Knowrob", 0)
        self.prolog_client = None

    @property
    def is_available(self) -> bool:
        return '/rosprolog/query' in rosservice.get_service_list()

    @property
    def is_connected(self) -> bool:
        return self.prolog_client is not None

    def connect(self):
        if self.is_available:
            self.prolog_client = Prolog()
            self.prolog_client.once(f"tripledb_load('package://iai_apartment/owl/iai-apartment.owl').")

    def query(self, designator: DesignatorDescription) -> DesignatorDescription:
        pass

    def query_pose_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        result = self.prolog_client.once(f"")
