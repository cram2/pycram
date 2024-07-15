import unittest

import roslaunch

from pycram.worlds.bullet_world import BulletWorld, Object
from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ParkArmsAction
from pycram.datastructures.enums import ObjectType, Arms, WorldMode
from pycram.failure_handling import Retry
from pycram.plan_failures import PlanFailure
from pycram.process_module import ProcessModule, simulated_robot
from pycram.robot_description import RobotDescription
from pycram.object_descriptors.urdf import ObjectDescription

extension = ObjectDescription.get_file_extension()


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
        cls.world = BulletWorld(WorldMode.DIRECT)
        cls.robot = Object(RobotDescription.current_robot_description.name, ObjectType.ROBOT, RobotDescription.current_robot_description.name + extension,
                           ObjectDescription)
        ProcessModule.execution_delay = True

    def setUp(self):
        self.world.reset_world()

    def test_retry_with_success(self):
        with simulated_robot:
            Retry(ParkArmsAction([Arms.LEFT]), max_tries=5).perform()

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
