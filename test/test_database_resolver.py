import os
import time
import unittest

import numpy as np

import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.resolver.location.database_location import DatabaseCostmapLocation
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
import sqlalchemy
import sqlalchemy.orm

pycrorm_uri = os.getenv('PYCRORM_URI')
if pycrorm_uri:
    pycrorm_uri = "mysql+pymysql://" + pycrorm_uri


@unittest.skipIf(pycrorm_uri is None,  "pycrorm database is not available.")
class DatabaseResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object

    @classmethod
    def setUpClass(cls) -> None:
        cls.world = BulletWorld("GUI")
        cls.milk = Object("milk", "milk", "milk.stl", position=[3, 3, 0.75])
        cls.robot = Object(robot_description.i.name, "pr2", robot_description.i.name + ".urdf")
        ProcessModule.execution_delay = False
        engine = sqlalchemy.create_engine("mysql+pymysql://pycrorm@localhost/pycrorm?charset=utf8mb4")

        cls.session = sqlalchemy.orm.Session(bind=engine)

    def test_costmap_no_obstacles(self):
        """Check if grasping a milk in the air works."""

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
        sample = next(iter(cml))

        with simulated_robot:
            action_designator.NavigateAction.Action(sample.pose).perform()
            action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
            time.sleep(5)
            action_designator.PickUpAction.Action(
                object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                arm=sample.reachable_arm, grasp=sample.grasp).perform()
            time.sleep(5)
    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")
        self.milk.set_position([-1.2, 1, 0.98])

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)

        for i in range(20):
            sample = next(iter(cml))
            with simulated_robot:
                action_designator.NavigateAction.Action(sample.pose).perform()
                action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
                time.sleep(5)
                try:
                    action_designator.PickUpAction.Action(
                        object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                        arm=sample.reachable_arm, grasp=sample.grasp).perform()
                except pycram.plan_failures.PlanFailure:
                    continue
                time.sleep(5)
                return
        raise pycram.plan_failures.PlanFailure()

    def tearDown(self) -> None:
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
