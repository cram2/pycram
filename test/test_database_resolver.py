import os
import unittest
import sqlalchemy
import sqlalchemy.orm
import pycram.plan_failures
from pycram import task
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.orm.base import Base
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.task import with_tree
from pycram.enums import ObjectType

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
class DatabaseResolverTestCase(unittest.TestCase,):
    world: BulletWorld
    milk: Object
    robot: Object
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

    @classmethod
    def setUpClass(cls) -> None:
        global pycrorm_uri
        cls.world = BulletWorld("DIRECT")
        cls.milk = Object("milk", "milk", "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        ProcessModule.execution_delay = False
        cls.engine = sqlalchemy.create_engine(pycrorm_uri)
        cls.session = sqlalchemy.orm.Session(bind=cls.engine)

    def setUp(self) -> None:
        self.world.reset_bullet_world()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session.commit()

    def tearDown(self) -> None:
        self.world.reset_bullet_world()
        pycram.task.reset_tree()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()
        cls.session.commit()
        cls.session.close()

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        with simulated_robot:
            action_designator.NavigateAction.Action(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            action_designator.MoveTorsoAction.Action(0.3).perform()
            action_designator.PickUpAction.Action(object_description.resolve(), "left", "front").perform()
            description.resolve().perform()

    def test_costmap_no_obstacles(self):
        """Check if grasping a milk in the air works."""
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "costmap_no_obstacles_test"
        pycram.task.task_tree.root.insert(self.session)

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
        sample = next(iter(cml))

        with simulated_robot:
            # action_designator.NavigateAction.Action(sample.pose).perform()
            action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
            action_designator.PickUpAction.Action(
                object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                arm=sample.reachable_arm, grasp=sample.grasp).perform()

    @unittest.skip
    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")

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


if __name__ == '__main__':
    unittest.main()
