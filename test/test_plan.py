import unittest

from pycram.designators.action_designator import *
from pycram.language import SequentialPlan, ParallelPlan


class TestPlan(unittest.TestCase):

    def test_plan_construction(self):
        sp = SequentialPlan(MoveTorsoActionDescription(None),
                            NavigateActionDescription(None))
        self.assertEqual(2, len(sp.root.children))
        self.assertEqual(3, len(sp.nodes))
        self.assertEqual(2, len(sp.edges))

    def test_plan_construction_nested(self):
        sp = SequentialPlan(MoveTorsoActionDescription(None),
                            MoveTorsoActionDescription(None),
                            ParallelPlan(NavigateActionDescription(None) ,
                                         NavigateActionDescription(None)))

        self.assertEqual(3, len(sp.root.children))
        self.assertEqual(6, len(sp.nodes))
        self.assertEqual(5, len(sp.edges))


class AlgebraTest(unittest.TestCase):

    def test_algebra(self):
        sp = SequentialPlan(MoveTorsoActionDescription(None),
                            NavigateActionDescription(None))

        sp.parameter_algebra()