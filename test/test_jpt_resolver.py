import os
import unittest

import enums
import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.designators.actions.actions import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.robot_descriptions import robot_description
from pycram.pose import Pose
from pycram.enums import ObjectType
import pycram.orm
import pycram.task
from pycram.task import with_tree
from pycram.designators.object_designator import ObjectDesignatorDescription

import sqlalchemy
import sqlalchemy.orm

# check if jpt is installed
jpt_installed = True
try:
    from probabilistic_model.learning.jpt.jpt import JPT
    from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
    from pycram.resolver.location.jpt_location import JPTCostmapLocation
except ImportError:
    jpt_installed = False

pycrorm_uri = os.getenv('PYCRORM_URI')
if pycrorm_uri:
    pycrorm_uri = "mysql+pymysql://" + pycrorm_uri
@unittest.skipIf(not jpt_installed, "probabilistic model is not installed. "
                                    "Install via 'pip install probabilistic_model")
class JPTResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session
    model: JPT

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
        self.learn_model()

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
        object_description = ObjectDesignatorDescription(types=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), "left", "front").perform()
            description.resolve().perform()

    def learn_model(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "costmap_no_obstacles_test"
        pycram.task.task_tree.root.insert(self.session)
        self.model = JPTCostmapLocation.fit_from_database(self.session)

    def test_costmap_no_obstacles(self):
        """Check if grasping a milk in the air works."""
        cml = JPTCostmapLocation(self.milk, model=self.model)
        sample = next(iter(cml))

        with simulated_robot:
            MoveTorsoActionPerformable(sample.torso_height).perform()
            PickUpActionPerformable(ObjectDesignatorDescription(types=["milk"]).resolve(), arm=sample.reachable_arm,
                                    grasp=sample.grasp).perform()


if __name__ == '__main__':
    unittest.main()
