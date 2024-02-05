import os
import unittest
import sqlalchemy
import sqlalchemy.orm
import pycram.plan_failures
from pycram import task
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator
from pycram.designators.actions.actions import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable
from pycram.orm.base import Base
from pycram.designators.object_designator import ObjectDesignatorDescription
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

    def setUp(self) -> None:
        self.world.reset_bullet_world()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self) -> None:
        self.world.reset_bullet_world()
        pycram.task.reset_tree()
        pycram.orm.base.ProcessMetaData.reset()
        self.session.rollback()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        self.session.close()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()

    @with_tree
    def plan(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), "left", "front").perform()
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
            MoveTorsoActionPerformable(sample.torso_height).perform()
            PickUpActionPerformable(ObjectDesignatorDescription(types=["milk"]).resolve(), arm=sample.reachable_arm,
                                    grasp=sample.grasp).perform()

    @unittest.skip
    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)

        for i in range(20):
            sample = next(iter(cml))
            with simulated_robot:
                NavigateActionPerformable(sample.pose).perform()
                MoveTorsoActionPerformable(sample.torso_height).perform()
                try:
                    PickUpActionPerformable(
                        ObjectDesignatorDescription(types=["milk"]).resolve(),
                        arm=sample.reachable_arm, grasp=sample.grasp).perform()
                except pycram.plan_failures.PlanFailure:
                    continue
                return
        kitchen.remove()
        raise pycram.plan_failures.PlanFailure()


if __name__ == '__main__':
    unittest.main()
