import unittest
from datetime import timedelta

from pycram.robot_plans  import ActionDescription
from pycram.robot_plans import ParkArmsAction, ParkArmsActionDescription
from pycram.datastructures.enums import ObjectType, Arms, WorldMode
from pycram.failure_handling import Retry
from pycram.failures import PlanFailure
from pycram.process_module import ProcessModule, simulated_robot
from pycram.robot_description import RobotDescription
from pycrap.ontologies import Robot



# start ik_and_description.launch
class DummyActionDesignator(ActionDescription):
    class Action(ActionDescription):
        def perform(self):
            raise PlanFailure("Dummy action failed")

    def __iter__(self):
        for _ in range(100):
            yield self.Action()


@unittest.skip
class FailureHandlingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(WorldMode.DIRECT)
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + extension)
        ProcessModule.execution_delay = timedelta(seconds=0.5)

    def setUp(self):
        self.world.reset_world()

    def test_retry_with_success(self):
        with simulated_robot:
            Retry(ParkArmsActionDescription([Arms.LEFT]), max_tries=5).perform()

    def test_retry_with_failure(self):
        with simulated_robot:
            with self.assertRaises(PlanFailure):
                Retry(DummyActionDesignator(), max_tries=5).perform()

    def tearDown(self):
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
