import os
import unittest
import time

import jpt
import numpy as np
import requests

from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.resolver.location.jpt_location import JPTCostmapLocation
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description

mlflow_uri = os.getenv('MLFLOW_TRACKING_URI')
if mlflow_uri is None:
    mlflow_uri = "http://127.0.0.1:5000"

try:
    response = requests.get(mlflow_uri, timeout=2)
except requests.exceptions.ConnectionError:
    response = None


@unittest.skipIf(response is None or (response.status_code != requests.codes.ok), "mlflow server is not available.")
class JPTResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object
    model: jpt.JPT

    @classmethod
    def setUpClass(cls) -> None:
        np.random.seed(420)
        cls.model = jpt.JPT.load(os.path.join(os.path.expanduser("~"), "Documents", "grasping.jpt"))
        cls.world = BulletWorld("GUI")
        cls.milk = Object("milk", "milk", "milk.stl", position=[3, 3, 0.75])
        cls.robot = Object(robot_description.i.name, "pr2", robot_description.i.name + ".urdf")
        ProcessModule.execution_delay = False

    def test_costmap_no_obstacles(self):
        cml = JPTCostmapLocation(self.milk, reachable_for=self.robot, model=self.model)
        sample = next(iter(cml))

        with simulated_robot:
            action_designator.NavigateAction.Action(sample.pose).perform()
            action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
            action_designator.PickUpAction.Action(
                object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                arm=sample.reachable_arm, grasp=sample.grasp).perform()

    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")
        self.milk.set_position([-1.2, 1, 0.98])
        cml = JPTCostmapLocation(self.milk, reachable_for=self.robot, model=self.model)
        sample = next(iter(cml))



        with simulated_robot:
            action_designator.NavigateAction.Action(sample.pose).perform()
            action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
            print(sample)
            time.sleep(10)
            action_designator.PickUpAction.Action(
                object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                arm=sample.reachable_arm, grasp=sample.grasp).perform()

    def tearDown(self) -> None:
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
