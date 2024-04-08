import random
import unittest
import time

import numpy as np
from random_events.events import Event

from pycram.designator import ObjectDesignatorDescription
from pycram.datastructures.enums import ObjectType
from pycram.designators.actions.actions import MoveTorsoActionPerformable
from pycram.plan_failures import PlanFailure
from pycram.process_module import simulated_robot
from pycram.resolver.probabilistic.probabilistic_action import MoveAndPickUp, GaussianCostmapModel
from bullet_world_testcase import BulletWorldTestCase
import plotly.graph_objects as go


class GaussianCostmapModelTestCase(unittest.TestCase):

    def test_create_model(self):
        gcm = GaussianCostmapModel()
        model = gcm.create_model()
        self.assertEqual(model.probability(gcm.center_event()), 0)
        self.assertEqual(len(model.variables), 4)
        # p_xy = model.marginal([gcm.relative_x, gcm.relative_y])
        # go.Figure(p_xy.plot(), p_xy.plotly_layout()).show()


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

        model = move_and_pick.ground_model()
        event = move_and_pick.events_from_occupancy_and_visibility_costmap()
        self.assertTrue(event.are_events_disjoint())
        self.assertIsNotNone(model)


    def test_move_and_pick_up(self):
        object_designator = ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve()
        move_and_pick = MoveAndPickUp(object_designator, arms=["left", "right"],
                                      grasps=["front", "left", "right", "top"])
        with simulated_robot:
            for action in move_and_pick:
                try:
                    MoveTorsoActionPerformable(0.3).perform()
                    action.perform()
                    return  # Success
                except PlanFailure as e:
                    continue
        raise AssertionError("No action performed successfully.")


if __name__ == '__main__':
    unittest.main()
