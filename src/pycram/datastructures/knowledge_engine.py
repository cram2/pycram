from .knowledge_source import KnowledgeSource


class KnowledgeEngine:
    def __init__(self):
        self.knowledge_sources = []

    def _init_sources(self):
        sources = KnowledgeSource.__subclasses__()
        for src in sources:
            self.knowledge_sources.append(src())
        self.knowledge_sources.sort(key=lambda x: x.priority)

    def query(self):
        ...
