import os
import unittest

import numpy as np
import requests

import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.robot_descriptions import robot_description
from pycram.pose import Pose

# check if jpt is installed
jpt_installed = True
try:
    import jpt
    from pycram.resolver.location.jpt_location import JPTCostmapLocation
except ImportError:
    jpt_installed = False

# check if mlflow is installed
mlflow_installed = True
try:
    import mlflow
except ImportError:
    mlflow_installed = False

mlflow_uri = os.getenv('MLFLOW_TRACKING_URI')
if mlflow_uri is None:
    mlflow_uri = "http://127.0.0.1:5000"

try:
    response = requests.get(mlflow_uri, timeout=2)
except requests.exceptions.ConnectionError:
    response = None


@unittest.skipIf(response is None or (response.status_code != requests.codes.ok), "mlflow server is not available.")
@unittest.skipIf(not jpt_installed, "jpt is not installed. Install via 'pip install pyjpt'")
@unittest.skipIf(not mlflow_installed, "mlflow is not installed. Install via 'pip install mlflow'")
class JPTResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object
    model: 'jpt.JPT'

    @classmethod
    def setUpClass(cls) -> None:
        np.random.seed(420)
        cls.model = mlflow.pyfunc.load_model(
            model_uri="mlflow-artifacts:/0/9150dd1fb353494d807261928cea6e8c/artifacts/grasping").unwrap_python_model() \
            .model
        cls.world = BulletWorld("DIRECT")
        cls.milk = Object("milk", "milk", "milk.stl", pose=Pose([3, 3, 0.75]))
        cls.robot = Object(robot_description.name, "pr2", robot_description.name + ".urdf")
        ProcessModule.execution_delay = False

    def test_costmap_no_obstacles(self):
        """Check if grasping a milk in the air works."""
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
        self.milk.set_pose(Pose([-1.2, 1, 0.98]))
        cml = JPTCostmapLocation(self.milk, reachable_for=self.robot, model=self.model)

        for i in range(20):
            sample = next(iter(cml))
            with simulated_robot:
                action_designator.NavigateAction.Action(sample.pose).perform()
                action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
                try:
                    action_designator.PickUpAction.Action(
                        object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                        arm=sample.reachable_arm, grasp=sample.grasp).perform()
                except pycram.plan_failures.PlanFailure:
                    continue
                return
        raise pycram.plan_failures.PlanFailure()

    def tearDown(self) -> None:
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
