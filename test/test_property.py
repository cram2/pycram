from pycram.datastructures.property import Property, And, Or, Not, ResolvedProperty
from pycram.datastructures.pose import PoseStamped
from knowledge_testcase import TestProperty, KnowledgeSourceTestCase
from pycram.failures import ReasoningError


class TestCaseProperty(KnowledgeSourceTestCase):
    def test_and_construct(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_3 = prop_1 & prop_2
        self.assertEqual(type(prop_3), And)
        self.assertEqual(prop_3.children, (prop_1, prop_2))

    def test_or_construct(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_3 = prop_1 | prop_2
        self.assertEqual(type(prop_3), Or)
        self.assertEqual(prop_3.children, (prop_1, prop_2))

    def test_not_construct(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_2 = ~prop_1
        self.assertEqual(type(prop_2), Not)
        self.assertEqual(prop_2.children, (prop_1,))

    def test_property_resolve(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3]))
        prop_3 = prop_1 & prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        self.assertTrue(resolved_property.resolved)
        self.assertEqual(type(resolved_property), And)
        self.assertEqual(resolved_property, prop_3)
        self.assertEqual(tuple((type(rp) for rp in resolved_property.children)),
                         (ResolvedProperty, ResolvedProperty))
        self.assertEqual(resolved_property.children[0].resolved_property_function, self.knowledge_source.test)
        self.assertEqual(resolved_property.children[1].resolved_property_function, self.knowledge_source.test)
        self.assertEqual(resolved_property.children[0].property_exception, ReasoningError)
        self.assertEqual(len(resolved_property.children), 2)
        self.assertTrue(resolved_property.children[0].is_leaf)
        self.assertTrue(resolved_property.children[1].is_leaf)

    def test_property_execute_true(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose1"))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose2"))
        prop_3 = prop_1 & prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        resolved_property()
        self.assertTrue(resolved_property.resolved)
        self.assertTrue(resolved_property.successful)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([1, 2, 3], frame="pose1"),
                                                       "pose2": PoseStamped.from_list([1, 2, 3], frame="pose2")})
        self.assertTrue(resolved_property.executed)

    def test_property_execute_false(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose1"))
        prop_2 = TestProperty(PoseStamped.from_list([9, 9, 9], frame="pose2"))
        prop_3 = prop_1 & prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        self.assertRaises(ReasoningError, resolved_property)
        self.assertTrue(resolved_property.resolved)
        self.assertFalse(resolved_property.successful)
        self.assertTrue(resolved_property.executed)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([1, 2, 3], frame="pose1"),
                                                       "pose2": PoseStamped.from_list([9, 9, 9], frame="pose2")})

    def test_property_execute_and_sparse(self):
        prop_1 = TestProperty(PoseStamped.from_list([9, 9, 9], frame="pose1"))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose2"))
        prop_3 = prop_1 & prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        self.assertRaises(ReasoningError, resolved_property)
        self.assertTrue(resolved_property.resolved)
        self.assertFalse(resolved_property.successful)
        self.assertFalse(resolved_property.children[1].successful)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([9, 9, 9], frame="pose1")})
        self.assertTrue(resolved_property.executed)

    def test_property_execute_or_false(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose1"))
        prop_2 = TestProperty(PoseStamped.from_list([9, 9, 9], frame="pose2"))
        prop_3 = prop_1 | prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        self.assertRaises(ReasoningError, resolved_property)
        self.assertTrue(resolved_property.resolved)
        self.assertFalse(resolved_property.successful)
        self.assertTrue(resolved_property.executed)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([1, 2, 3], frame="pose1"),
                                                       "pose2": PoseStamped.from_list([9, 9, 9], frame="pose2")})

    def test_property_execute_or_true(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose1"))
        prop_2 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose2"))
        prop_3 = prop_1 | prop_2
        resolved_property = self.knowledge_engine.resolve_properties(prop_3)
        resolved_property()
        self.assertTrue(resolved_property.resolved)
        self.assertTrue(resolved_property.successful)
        self.assertTrue(resolved_property.executed)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([1, 2, 3], frame="pose1"),
                                                       "pose2": PoseStamped.from_list([1, 2, 3], frame="pose2")})

    def test_property_execute_not_true(self):
        prop_1 = TestProperty(PoseStamped.from_list([1, 2, 3], frame="pose1"))
        prop_2 = ~prop_1
        resolved_property = self.knowledge_engine.resolve_properties(prop_2)
        resolved_property()
        self.assertTrue(resolved_property.resolved)
        self.assertFalse(resolved_property.successful)
        self.assertTrue(resolved_property.executed)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([1, 2, 3], frame="pose1")})

    def test_property_execute_not_false(self):
        prop_1 = TestProperty(PoseStamped.from_list([9, 9, 9], frame="pose1"))
        prop_2 = ~prop_1
        resolved_property = self.knowledge_engine.resolve_properties(prop_2)
        self.assertRaises(ReasoningError, resolved_property)
        self.assertTrue(resolved_property.resolved)
        self.assertFalse(resolved_property.successful)
        self.assertTrue(resolved_property.executed)
        self.assertEqual(resolved_property.variables, {"pose1": PoseStamped.from_list([9, 9, 9], frame="pose1")})