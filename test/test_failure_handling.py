import unittest

import roslaunch

from pycram.bullet_world import BulletWorld, Object
from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ParkArmsAction
from pycram.enums import ObjectType, Arms
from pycram.failure_handling import Retry
from pycram.plan_failures import PlanFailure
from pycram.process_module import ProcessModule, simulated_robot
from pycram.robot_descriptions import robot_description


# start ik_and_description.launch
class DummyActionDesignator(ActionDesignatorDescription):
    class Action(ActionDesignatorDescription.Action):
        def perform(self):
            raise PlanFailure("Dummy action failed")

    def __iter__(self):
        for _ in range(100):
            yield self.Action()


class FailureHandlingTest(unittest.TestCase):
    world: BulletWorld
    process: roslaunch.scriptapi.ROSLaunch

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        ProcessModule.execution_delay = True

    def setUp(self):
        self.world.reset_bullet_world()

    def test_retry_with_success(self):
        with simulated_robot:
            Retry(ParkArmsAction([Arms.LEFT]), max_tries=5).perform()

    def test_retry_with_failure(self):
        with simulated_robot:
            with self.assertRaises(PlanFailure):
                Retry(DummyActionDesignator(), max_tries=5).perform()

    def tearDown(self):
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
