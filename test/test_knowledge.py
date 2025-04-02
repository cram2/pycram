import unittest
from itertools import permutations

from pycram.testing import BulletWorldTestCase
from knowledge_testcase import KnowledgeSourceTestCase, TestProperty, KnowledgeBulletTestCase
from pycram.datastructures.enums import Arms, Grasp, ObjectType, TorsoState, GripperState
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import PoseStamped, GraspDescription
from pycram.designators.action_designator import PickUpAction, PickUpAction, OpenAction, MoveTorsoAction, \
    PickUpActionDescription, OpenActionDescription
from pycram.designators.object_designator import BelieveObject, ObjectPart
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
        prop = TestProperty(PoseStamped.from_list([1, 2, 3]))
        self.assertEqual(self.knowledge_engine.find_source_for_property(prop), self.knowledge_source)

@unittest.skip
class TestKnowledgeEngineBeliefState(KnowledgeBulletTestCase):
    def test_match_by_name(self):
        params = {"arm": 1, "leg": "left"}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_name(params, desig)
        self.assertEqual({"arm": 1}, matched)

    def test_match_by_name_no_match(self):
        params = {"test": 1, "leg": "left"}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_name(params, desig)
        self.assertEqual({}, matched)

    def test_match_by_type(self):
        params = {"test": Arms.RIGHT, "leg": "left"}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_type(params, desig)
        self.assertEqual({"arm": Arms.RIGHT}, matched)

    def test_match_by_type_no_match(self):
        params = {"test": {"test": 1}, "leg": "left"}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine._match_by_type(params, desig)
        self.assertEqual({}, matched)

    def test_match_reasoned_parameter(self):
        params = {"arm": Arms.RIGHT, "leg": "left"}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine.match_reasoned_parameter(params, desig)
        self.assertEqual({"arm": Arms.RIGHT}, matched)

    def test_match_reasoned_parameter_full(self):
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        params = {"arm": Arms.RIGHT, "grasp_description": grasp_description}
        desig = PickUpActionDescription(BelieveObject(names=["milk"]))
        matched = self.knowledge_engine.match_reasoned_parameter(params, desig)
        self.assertEqual({"arm": Arms.RIGHT, "grasp_description": grasp_description}, matched)


class TestParameterInference(KnowledgeBulletTestCase):
    @unittest.skip
    def test_pickup_arm(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PickUpActionDescription(test_object, [Arms.RIGHT])
        desig = partial_desig.resolve()
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        self.assertEqual(desig.grasp_description, None)

    @unittest.skip
    def test_pickup_grasp(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PickUpActionDescription(test_object, [Arms.RIGHT])
        desig = partial_desig.resolve()
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        self.assertEqual(desig.grasp_description, None)

    def test_open_gripper(self):
        self.robot.set_pose(PoseStamped.from_list([-0.192, 1.999, 0], [0, 0, 0.8999, -0.437]))
        self.robot.set_joint_position("torso_lift_joint", 0.3)
        env_object = BelieveObject(names=["kitchen"]).resolve()
        handle_desig = ObjectPart(["kitchen_island_middle_upper_drawer_handle"], env_object)
        partial_desig = OpenActionDescription(handle_desig, [Arms.RIGHT])
        desig = partial_desig.resolve()
        self.assertEqual(desig.arm, Arms.RIGHT)


class TestReasoningInstance(KnowledgeSourceTestCase):
    pass
