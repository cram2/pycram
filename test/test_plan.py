import unittest

from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized
from random_events.product_algebra import SimpleEvent, Event

from pycram.designators.action_designator import *
from pycram.language import SequentialPlan, ParallelPlan
from pycram.parameterizer import Parameterizer
from pycram.testing import BulletWorldTestCase


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


class AlgebraTest(BulletWorldTestCase):

    def test_algebra(self):
        sp = SequentialPlan(MoveTorsoActionDescription(None),
                            NavigateActionDescription(None),
                            MoveTorsoActionDescription(None))

        p = Parameterizer(sp)
        distribution = fully_factorized(p.variables, means={v: 0 for v in p.variables},
                                        variances={v: 1 for v in p.variables})

        conditions = []
        for state in TorsoState:
            v1 = p.get_variable("MoveTorsoAction_0.torso_state")
            v2 = p.get_variable("MoveTorsoAction_2.torso_state")
            se = SimpleEvent({v1: state, v2: state})
            conditions.append(se)
        condition = Event(*conditions)
        condition.fill_missing_variables(p.variables)

        conditional, p_c = distribution.conditional(condition)
        sample = distribution.sample(1)
        print(sample)
        resolved = p.plan_from_sample(conditional, sample[0])

        resolved.perform()

