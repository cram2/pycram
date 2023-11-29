import unittest
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.new_language import Sequential, Language, Parallel, TryAll, TryInOrder

from anytree import RenderTree


class LanguageTestCase(unittest.TestCase):

    def test_inheritance(self):
        act = NavigateAction([Pose()])
        self.assertTrue(issubclass(act.__class__, Language))

    def test_simplify_tree(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertEqual(len(plan.children), 3)
        self.assertEqual(plan.height, 1)

    def test_sequential_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertTrue(isinstance(plan, Sequential))
        self.assertEqual(len(plan.children), 3)

    def test_parallel_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act | act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 3)

    def test_try_in_order_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act - act2 - act3
        self.assertTrue(isinstance(plan, TryInOrder))
        self.assertEqual(len(plan.children), 3)

    def test_try_all_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act ^ act2 ^ act3
        self.assertTrue(isinstance(plan, TryAll))
        self.assertEqual(len(plan.children), 3)

    def test_combination_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 2)
        self.assertEqual(plan.height, 2)


if __name__ == '__main__':
    unittest.main()
