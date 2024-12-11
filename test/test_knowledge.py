from pycram.testing import BulletWorldTestCase
from knowledge_testcase import KnowledgeSourceTestCase, TestProperty, KnowledgeBulletTestCase
from pycram.datastructures.enums import Arms, Grasp, ObjectType
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable, OpenAction
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
        params = {"test": {"test": 1}, "leg": "left"}
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


class TestPartialDesignator(KnowledgeBulletTestCase):
    def test_partial_desig_construction(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PartialDesignator(PickUpActionPerformable, test_object, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpActionPerformable)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": test_object,
                                                "grasp": None, 'prepose_distance': None})

    def test_partial_desig_construction_none(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpActionPerformable)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": None,
                                                "grasp": None, 'prepose_distance': None})

    def test_partial_desig_call(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        new_partial_desig = partial_desig(grasp=Grasp.FRONT)
        self.assertEqual(new_partial_desig.performable, PickUpActionPerformable)
        self.assertEqual({"arm": Arms.RIGHT, "grasp": Grasp.FRONT, "object_designator": None,
                          'prepose_distance': None}, new_partial_desig.kwargs)

    def test_partial_desig_missing_params(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        missing_params = partial_desig.missing_parameter()
        self.assertTrue("object_designator" in missing_params and "grasp" in missing_params)

        new_partial = partial_desig(grasp=Grasp.FRONT)
        missing_params = new_partial.missing_parameter()
        self.assertEqual(['object_designator', 'prepose_distance'], missing_params)

    def test_is_iterable(self):
        self.assertTrue(PartialDesignator._is_iterable([1, 2, 3]))
        self.assertFalse(PartialDesignator._is_iterable(1))

    def test_partial_desig_permutations(self):
        l1 = [1, 2]
        l2 = [Arms.RIGHT, Arms.LEFT]
        permutations = PartialDesignator.generate_permutations([l1, l2])
        self.assertEqual([(1, Arms.RIGHT), (1, Arms.LEFT), (2, Arms.RIGHT), (2, Arms.LEFT)], list(permutations))

    def test_partial_desig_iter(self):
        test_object = BelieveObject(names=["milk"])
        test_object_resolved = test_object.resolve()
        partial_desig = PartialDesignator(PickUpActionPerformable, test_object, arm=[Arms.RIGHT, Arms.LEFT])
        performables = list(partial_desig(grasp=[Grasp.FRONT, Grasp.TOP]))
        self.assertEqual(4, len(performables))
        self.assertTrue(all([isinstance(p, PickUpActionPerformable) for p in performables]))
        self.assertEqual([p.arm for p in performables], [Arms.RIGHT, Arms.RIGHT, Arms.LEFT, Arms.LEFT])
        self.assertEqual([p.grasp for p in performables], [Grasp.FRONT, Grasp.TOP, Grasp.FRONT, Grasp.TOP])
        self.assertEqual([p.object_designator for p in performables], [test_object_resolved] * 4)


class TestParameterInference(KnowledgeBulletTestCase):
    def test_pickup_arm(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PickUpAction(test_object, [Arms.RIGHT])
        desig = partial_desig.resolve()
        self.assertEqual(desig.grasp, Grasp.FRONT)

    def test_pickup_grasp(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PickUpAction(test_object, [Arms.RIGHT])
        desig = partial_desig.resolve()
        self.assertEqual(desig.grasp, Grasp.FRONT)

    def test_open_gripper(self):
        self.robot.set_pose(Pose([-0.192, 1.999, 0], [0, 0, 0.8999, -0.437]))
        self.robot.set_joint_position("torso_lift_joint", 0.3)
        env_object = BelieveObject(names=["kitchen"]).resolve()
        handle_desig = ObjectPart(["kitchen_island_middle_upper_drawer_handle"], env_object)
        partial_desig = OpenAction(handle_desig, [Arms.RIGHT])
        desig = partial_desig.resolve()
        self.assertEqual(desig.arm, Arms.RIGHT)


class TestReasoningInstance(KnowledgeSourceTestCase):
    pass
