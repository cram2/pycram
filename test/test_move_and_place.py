import random
import unittest

import numpy as np

from bullet_world_testcase import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType, Arms, Grasp
from pycram.datastructures.pose import Pose
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import MoveTorsoActionPerformable, NavigateActionPerformable, \
    PickUpActionPerformable
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import (MoveAndPlace)
from pycram.failures import PlanFailure
from pycram.process_module import simulated_robot


class MoveAndPlaceTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

    def test_with_mode(self):
        object_designator = ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve()
        target_location = Pose([1.3, 1, 0.9], [0, 0, 0, 1])
        designator = MoveAndPlace(object_designator, target_location)

        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_designator, Arms.LEFT, Grasp.FRONT).perform()
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
