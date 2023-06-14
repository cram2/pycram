import os
import unittest

import sqlalchemy
import sqlalchemy.orm

import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot

from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description

# check if jpt is installed
jpt_installed = True
try:
    import jpt
    from pycram.resolver.location.database_location import DatabaseCostmapLocation
except ImportError:
    jpt_installed = False


pycrorm_uri = os.getenv('PYCRORM_URI')
if pycrorm_uri:
    pycrorm_uri = "mysql+pymysql://" + pycrorm_uri


@unittest.skipIf(pycrorm_uri is None, "pycrorm database is not available.")
@unittest.skipIf(not jpt_installed, "jpt is not installed but needed for the definition of DatabaseCostmapLocations. "
                                    "Install via 'pip install pyjpt'")
class DatabaseResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object

    @classmethod
    def setUpClass(cls) -> None:
        global pycrorm_uri
        cls.world = BulletWorld("DIRECT")
        cls.milk = Object("milk", "milk", "milk.stl", position=[3, 3, 0.75])
        cls.robot = Object(robot_description.i.name, "pr2", robot_description.i.name + ".urdf")
        ProcessModule.execution_delay = False
        engine = sqlalchemy.create_engine(pycrorm_uri)

        cls.session = sqlalchemy.orm.Session(bind=engine)

    def test_costmap_no_obstacles(self):
        """Check if grasping a milk in the air works."""

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
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

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)

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
