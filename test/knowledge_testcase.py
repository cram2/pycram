import unittest
from abc import abstractmethod

from pycram.testing import BulletWorldTestCase
from pycram.datastructures.dataclasses import ReasoningResult
from pycram.datastructures.pose import Pose
from pycram.datastructures.property import Property
from pycram.failures import ReasoningError
from pycram.knowledge.knowledge_source import KnowledgeSource
from pycram.knowledge.knowledge_engine import KnowledgeEngine


class TestProperty(Property):
    """
    A Mock Property for testing the property implementation
    """
    property_exception = ReasoningError

    def __init__(self, pose: Pose):
        super().__init__(None, None)
        self.pose = pose

    @abstractmethod
    def test(self, pose: Pose) -> ReasoningResult:
        raise NotImplementedError


class TestKnowledge(KnowledgeSource, TestProperty):
    """
    A Mock KnowledgeSource for testing the knowledge_source implementation
    """
    def __init__(self):
        super().__init__("test_source", 1)
        self.parameter = {}

    def is_available(self):
        return True

    def is_connected(self):
        return True

    def connect(self):
        pass

    def clear_state(self):
        self.parameter = {}

    def test(self, pose: Pose) -> ReasoningResult:
        return ReasoningResult(Pose([1, 2, 3]).dist(pose) < 0.1, {pose.frame: pose})


class KnowledgeSourceTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.knowledge_engine = KnowledgeEngine()
        cls.knowledge_source = list(filter(lambda x: type(x) == TestKnowledge, cls.knowledge_engine.knowledge_sources))[
            0]


class KnowledgeBulletTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.knowledge_engine = KnowledgeEngine()
        cls.knowledge_source = list(filter(lambda x: type(x) == TestKnowledge, cls.knowledge_engine.knowledge_sources))[
            0]
