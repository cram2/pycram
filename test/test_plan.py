import time
import unittest

from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized
from pycram.process_module import simulated_robot
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
        distribution = p.create_fully_factorized_distribution()

        conditions = []
        for state in TorsoState:
            v1 = p.get_variable("MoveTorsoAction_0.torso_state")
            v2 = p.get_variable("MoveTorsoAction_2.torso_state")
            se = SimpleEvent({v1: state, v2: state})
            conditions.append(se)


        condition = Event(*conditions)
        condition.fill_missing_variables(p.variables)

        navigate_condition = SimpleEvent({
            p.get_variable("NavigateAction_1.target_location.pose.position.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.x"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.y"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.w"): 1
        })
        navigate_condition.fill_missing_variables(p.variables)
        condition &= navigate_condition.as_composite_set()

        condition &= p.create_restrictions().as_composite_set()

        conditional, p_c = distribution.conditional(condition)
        sample = distribution.sample(1)
        resolved = p.plan_from_sample(conditional, sample[0])
        with simulated_robot:
            resolved.perform()
            time.sleep(100)