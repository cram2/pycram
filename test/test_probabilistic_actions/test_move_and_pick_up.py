import random
import unittest

import numpy as np

from pycram.designator import ObjectDesignatorDescription
from pycram.enums import ObjectType
from pycram.plan_failures import PlanFailure
from pycram.process_module import simulated_robot
from pycram.resolver.probabilistic.probabilistic_action import MoveAndPickUp
from ..bullet_world_testcase import BulletWorldTestCase
import plotly.graph_objects as go


class MoveAndPickUpTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

    def test_grounding(self):
        object_designator = ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=["left", "right"],
                                      grasps=["front", "left", "right", "top"])

        # p_xy = move_and_pick.policy.marginal([move_and_pick.variables.relative_x, move_and_pick.variables.relative_y])
        # fig = go.Figure(p_xy.plot())
        # fig.show()
        model = move_and_pick.ground_model()
        self.assertIsNotNone(model)

    def test_move_and_pick_up(self):
        object_designator = ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=["left", "right"],
                                      grasps=["front", "left", "right", "top"])
        with simulated_robot:
            for action in move_and_pick:
                try:
                    action.perform()
                    return  # Success
                except PlanFailure as e:
                    continue
        raise AssertionError("No action performed successfully.")


if __name__ == '__main__':
    unittest.main()
