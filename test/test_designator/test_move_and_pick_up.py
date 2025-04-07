import random
import unittest

import numpy as np
from random_events.variable import Continuous, Symbolic
from sortedcontainers import SortedSet

from pycram.testing import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType, Arms, Grasp, TorsoState
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import MoveTorsoAction
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import (MoveAndPickUp,
                                                                                           Arms as PMArms,
                                                                                           Grasp as PMGrasp)
from pycram.failures import PlanFailure
from pycram.process_module import simulated_robot
from pycrap.ontologies import Milk


@unittest.skip("Skip this test until PM is upgraded")
class MoveAndPickUpTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

    @unittest.skip
    def test_variables(self):
        object_designator = ObjectDesignatorDescription(types=[Milk]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=[Arms.LEFT, Arms.RIGHT],
                                      grasps=[Grasp.FRONT, Grasp.LEFT, Grasp.RIGHT, Grasp.TOP, Grasp.BOTTOM, Grasp.BACK])
        result = SortedSet([Symbolic("arm", PMArms), Symbolic("grasp", PMGrasp),
                            Continuous("relative_x"), Continuous("relative_y")])
        all_variables = move_and_pick.all_variables()
        self.assertEqual(all_variables, result)

    @unittest.skip
    def test_grounding(self):
        object_designator = ObjectDesignatorDescription(types=[Milk]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=[Arms.LEFT, Arms.RIGHT],
                                      grasps=[Grasp.FRONT, Grasp.LEFT, Grasp.RIGHT, Grasp.TOP, Grasp.BOTTOM, Grasp.BACK])

        model = move_and_pick.ground_model()
        event = move_and_pick.events_from_occupancy_and_visibility_costmap()
        self.assertTrue(event.is_disjoint())
        self.assertIsNotNone(model)

    @unittest.skip
    def test_move_and_pick_up_with_mode(self):
        object_designator = ObjectDesignatorDescription(types=[Milk]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=[Arms.LEFT, Arms.RIGHT],
                                      grasps=[Grasp.FRONT, Grasp.LEFT, Grasp.RIGHT, Grasp.TOP, Grasp.BOTTOM, Grasp.BACK])
        with simulated_robot:
            for action in move_and_pick.iter_with_mode():
                try:
                    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
                    action.perform()
                    return  # Success
                except PlanFailure as e:
                    continue
        raise AssertionError("No action performed successfully.")


if __name__ == '__main__':
    unittest.main()
