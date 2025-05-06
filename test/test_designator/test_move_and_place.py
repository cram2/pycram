import random
import unittest

import numpy as np

from pycram.testing import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType, Arms, Grasp, TorsoState
from pycram.datastructures.pose import PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import NavigateAction, \
    PickUpAction, MoveTorsoAction
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import (MoveAndPlace)
from pycram.failures import PlanFailure
from pycram.process_module import simulated_robot
from pycrap.ontologies import Milk


class MoveAndPlaceTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

    @unittest.skip
    def test_with_mode(self):
        object_designator = ObjectDesignatorDescription(types=[Milk]).resolve()
        target_location = PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])
        designator = MoveAndPlace(object_designator, target_location)

        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
            PickUpAction(object_designator, Arms.LEFT, Grasp.FRONT, 0.03).perform()
            with simulated_robot:
                for action in designator:
                    try:
                        action.perform()
                        return  # Success
                    except PlanFailure as e:
                        continue
            raise AssertionError("No action performed successfully.")

if __name__ == '__main__':
    unittest.main()
