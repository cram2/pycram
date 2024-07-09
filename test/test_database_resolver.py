import os
import unittest
import sqlalchemy
import sqlalchemy.orm
import pycram.plan_failures
from pycram.world_concepts.world_object import Object
from pycram.datastructures.world import World
from pycram.designators import action_designator
from pycram.designators.action_designator import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable, PlaceActionPerformable
from pycram.orm.base import Base
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose
from pycram.robot_description import RobotDescription
from pycram.tasktree import with_tree
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.specialized_designators.location.database_location import DatabaseCostmapLocation
from pycram.worlds.bullet_world import BulletWorld

pycrorm_uri = os.getenv('PYCRORM_URI')
if pycrorm_uri:
    pycrorm_uri = "mysql+pymysql://" + pycrorm_uri


@unittest.skipIf(pycrorm_uri is None, "pycrorm database is not available.")
class DatabaseResolverTestCase(unittest.TestCase,):
    world: World
    milk: Object
    robot: Object
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

    @classmethod
    def setUpClass(cls) -> None:
        global pycrorm_uri
        cls.world = BulletWorld(WorldMode.DIRECT)
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, RobotDescription.current_robot_description.name + ".urdf")
        ProcessModule.execution_delay = False
        cls.engine = sqlalchemy.create_engine(pycrorm_uri)

    def setUp(self) -> None:
        self.world.reset_world()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self) -> None:
        self.world.reset_world()
        pycram.tasktree.reset_tree()
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
        pycram.tasktree.task_tree.root.insert(self.session)

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
        sample = next(iter(cml))

        with simulated_robot:
            MoveTorsoActionPerformable(sample.torso_height).perform()
            PickUpActionPerformable(ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve(), arm=sample.reachable_arm,
                                    grasp=sample.grasp).perform()

    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "costmap_with_obstacles_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        self.world.reset_current_world()

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
        sample = next(iter(cml))

        with simulated_robot:
            NavigateActionPerformable(sample.pose).perform()
            MoveTorsoActionPerformable(sample.torso_height).perform()
            try:
                PickUpActionPerformable(
                    ObjectDesignatorDescription(types=[ObjectType.MILK]).resolve(),
                    arm=sample.reachable_arm, grasp=sample.grasp).perform()
            except pycram.plan_failures.PlanFailure as p:
                kitchen.remove()
                raise p
        kitchen.remove()

    def test_object_at_different_location(self):
        kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        self.plan()

        pycram.orm.base.ProcessMetaData().description = "object_at_different_location_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        self.world.reset_current_world()

        new_milk = Object("new_milk", ObjectType.MILK, "milk.stl", pose=Pose([-1.45, 2.5, 0.95]))
        cml = DatabaseCostmapLocation(new_milk, self.session, reachable_for=self.robot)

        sample = next(iter(cml))
        with simulated_robot:
            NavigateActionPerformable(sample.pose).perform()
            MoveTorsoActionPerformable(sample.torso_height).perform()
            try:
                PickUpActionPerformable(
                    ObjectDesignatorDescription(names=["new_milk"], types=[ObjectType.MILK]).resolve(),
                    arm=sample.reachable_arm, grasp=sample.grasp).perform()
            except pycram.plan_failures.PlanFailure as p:
                new_milk.remove()
                kitchen.remove()
                raise p
            PlaceActionPerformable(ObjectDesignatorDescription(names=["new_milk"], types=[ObjectType.MILK]).resolve(),
                                   arm=sample.reachable_arm, target_location=Pose([-1.45, 2.5, 0.95])).perform()
        new_milk.remove()
        kitchen.remove()

    @unittest.skip
    def test_multiple_objects(self):
        kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        new_milk = Object("new_milk", ObjectType.MILK, "milk.stl", pose=Pose([-1.45, 2.5, 0.9]))

        object_description = ObjectDesignatorDescription(names=["milk"])
        object_description_new_milk = ObjectDesignatorDescription(names=["new_milk"])
        description = action_designator.PlaceAction(object_description_new_milk, [Pose([-1.45, 2.5, 0.9], [0, 0, 0, 1])], ["left"])
        with simulated_robot:
            NavigateActionPerformable(Pose([1, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), "left", "front").perform()
            PlaceActionPerformable(object_description.resolve(), "left", Pose([1.3, 1, 0.9], [0, 0, 0, 1])).perform()
            NavigateActionPerformable(Pose([-1.75, 1.9, 0], [0, 0, 0, 1])).perform()
            PickUpActionPerformable(object_description_new_milk.resolve(), "left", "front").perform()
            description.resolve().perform()

        pycram.orm.base.ProcessMetaData().description = "multiple_objects_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        self.world.reset_current_world()

        cml = DatabaseCostmapLocation(self.milk, self.session, reachable_for=self.robot)
        cml_new_milk = DatabaseCostmapLocation(new_milk, self.session, reachable_for=self.robot)

        dcls = [cml, cml_new_milk]
        for dcl in dcls:
            sample = next(iter(dcl))
            with (simulated_robot):
                NavigateActionPerformable(sample.pose).perform()
                MoveTorsoActionPerformable(sample.torso_height).perform()
                try:
                    if dcl.target.name == "milk":
                        PickUpActionPerformable(
                            ObjectDesignatorDescription(names=["milk"], types=[ObjectType.MILK]).resolve(),
                            arm=sample.reachable_arm, grasp=sample.grasp).perform()
                    else:
                        PickUpActionPerformable(
                            ObjectDesignatorDescription(names=["new_milk"], types=[ObjectType.MILK]).resolve(),
                            arm=sample.reachable_arm, grasp=sample.grasp).perform()
                except pycram.plan_failures.PlanFailure as p:
                    new_milk.remove()
                    kitchen.remove()
                    raise p
                PlaceActionPerformable(ObjectDesignatorDescription(names=[dcl.target.name],
                                                                   types=[ObjectType.MILK]).resolve(),
                                       arm=sample.reachable_arm, target_location=Pose([dcl.target.pose.position.x,
                                                                                       dcl.target.pose.position.y,
                                                                                       dcl.target.pose.position.z])
                                       ).perform()
        new_milk.remove()
        kitchen.remove()


if __name__ == '__main__':
    unittest.main()
