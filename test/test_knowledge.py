from bullet_world_testcase import BulletWorldTestCase
from knowledge_testcase import KnowledgeSourceTestCase, TestProperty, KnowledgeBulletTestCase
from pycram.datastructures.enums import Arms, Grasp
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import PickUpAction
from pycram.designators.object_designator import BelieveObject
from pycram.knowledge.knowledge_engine import KnowledgeEngine

class TestKnowledgeSource(KnowledgeSourceTestCase):
    def test_knowledge_source_construct(self):
        self.assertEqual(self.knowledge_source.name, "test_source")
        self.assertEqual(self.knowledge_source.priority, 1)

    def test_available(self):
        self.assertTrue(self.knowledge_source.is_available)

    def test_connected(self):
        self.assertTrue(self.knowledge_source.is_connected)

    def test_init(self):
        self.assertTrue(type(self.knowledge_source) in
                        [type(self.knowledge_source) for s in self.knowledge_engine.knowledge_sources])


class TestKnowledgeEngine(KnowledgeSourceTestCase):
    def test_singleton(self):
        self.assertEqual(self.knowledge_engine, KnowledgeEngine())

    def test_init(self):
        self.assertTrue(self.knowledge_engine._initialized)

    def test_init_sources(self):
        self.assertTrue(len(self.knowledge_engine.knowledge_sources) > 0)

    def test_update_sources(self):
        self.knowledge_engine.update_sources()
        self.assertTrue(self.knowledge_source.is_connected)

    def test_find_source(self):
        prop = TestProperty(Pose([1, 2, 3]))
        self.assertEqual(self.knowledge_engine.find_source_for_property(prop), self.knowledge_source)


class TestKnowledgeEngineBeliefState(KnowledgeBulletTestCase):
    def test_match_by_name(self):
        params = {"arm": 1, "leg": "left"}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_name(params, desig)
        self.assertEqual({"arm": 1}, matched)

    def test_match_by_name_no_match(self):
        params = {"test": 1, "leg": "left"}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_name(params, desig)
        self.assertEqual({}, matched)

    def test_match_by_type(self):
        params = {"test": Arms.RIGHT, "leg": "left"}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_type(params, desig)
        self.assertEqual({"arm": Arms.RIGHT}, matched)

    def test_match_by_type_no_match(self):
        params = {"test": 1, "leg": "left"}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_type(params, desig)
        self.assertEqual({}, matched)

    def test_match_reasoned_parameter(self):
        params = {"arm": Arms.RIGHT, "leg": "left"}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine.match_reasoned_parameter(params, desig)
        self.assertEqual({"arm": Arms.RIGHT}, matched)

    def test_match_reasoned_parameter_full(self):
        params = {"arm": Arms.RIGHT, "gasp": Grasp.FRONT}
        desig = PickUpAction(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine.match_reasoned_parameter(params, desig)
        self.assertEqual({"arm": Arms.RIGHT, "grasp": Grasp.FRONT}, matched)

class TestReasoningInstance(KnowledgeSourceTestCase):
    pass
