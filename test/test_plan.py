from pycram.language import SequentialPlan, ParallelPlan
from pycram.designators.action_designator import *
import unittest

class TestPlan(unittest.TestCase):

    def test_plan_construction(self):
        sp = SequentialPlan()
        with sp:
            MoveTorsoActionDescription(None)
            NavigateActionDescription(None)
        self.assertEquals(2, len(sp.root.children))
        self.assertEquals(3, len(sp.nodes))
        self.assertEquals(2, len(sp.edges))

    def test_plan_construction_nested(self):
        sp = SequentialPlan()
        with sp:
            MoveTorsoActionDescription(None)
            MoveTorsoActionDescription(None)
            with ParallelPlan():
                NavigateActionDescription(None)
                NavigateActionDescription(None)

        self.assertEquals(3, len(sp.root.children))
        self.assertEquals(6, len(sp.nodes))
        self.assertEquals(5, len(sp.edges))