import os
import time
import unittest

import numpy as np
import pandas as pd
from anytree import RenderTree
from probabilistic_model.probabilistic_circuit.probabilistic_circuit import DeterministicSumUnit, ProbabilisticCircuit
from random_events.variables import Continuous

import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.designators.actions.actions import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.robot_descriptions import robot_description
from pycram.pose import Pose, Transform
from pycram.enums import ObjectType, Grasp, Arms
import pycram.orm
import pycram.task
from pycram.task import with_tree
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.bullet_world_reasoning import visible
from bullet_world_testcase import BulletWorldTestCase

import plotly.graph_objects as go
import sqlalchemy
import sqlalchemy.orm

from pycram.local_transformer import LocalTransformer
from pycram.resolver.location.jpt_location import JPTCostmapLocation, GaussianCostmapModel, FaceAtPerformable, \
    FunkyPickUpAction

# check if jpt is installed
jpt_installed = True
try:
    from probabilistic_model.learning.jpt.jpt import JPT
    from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe

except ImportError as e:
    print(e)
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
        # VizMarkerPublisher(interval=0.1)
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

    def test_costmap_with_obstacles(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")
        self.world.reset_bullet_world()

        cml = JPTCostmapLocation(self.milk, model=self.model)
        sample = next(iter(cml))

        with simulated_robot:
            NavigateActionPerformable(sample.pose).perform()

            MoveTorsoActionPerformable(sample.torso_height).perform()

            try:
                PickUpActionPerformable(
                    ObjectDesignatorDescription(types=["milk"]).resolve(),
                    arm=sample.reachable_arm, grasp=sample.grasp).perform()

            except pycram.plan_failures.PlanFailure as p:
                kitchen.remove()
                raise p
        kitchen.remove()

    def test_object_at_different_location(self):
        kitchen = Object("kitchen", "environment", "kitchen.urdf")
        self.world.reset_bullet_world()

        new_milk = Object("new_milk", "milk", "milk.stl", pose=Pose([-1.45, 2.5, 0.95]))
        cml = JPTCostmapLocation(new_milk, model=self.model)

        sample = next(iter(cml))
        with simulated_robot:
            NavigateActionPerformable(sample.pose).perform()
            MoveTorsoActionPerformable(sample.torso_height).perform()
            try:
                PickUpActionPerformable(
                    ObjectDesignatorDescription(names=["new_milk"], types=["milk"]).resolve(),
                    arm=sample.reachable_arm, grasp=sample.grasp).perform()
            except pycram.plan_failures.PlanFailure as p:
                raise p


class FacingTestCase(BulletWorldTestCase):

    def test_facing(self):

        with simulated_robot:
            FaceAtPerformable(self.robot.pose, self.milk.pose).perform()
            milk_in_robot_frame = LocalTransformer().transform_to_object_frame(self.milk.pose, self.robot)
            self.assertAlmostEqual(milk_in_robot_frame.position.y, 0.)


class TruncatedNormalTestCase(unittest.TestCase):

    def test_normal_costmap(self):
        gaussian_model = GaussianCostmapModel(0.1, 0.5)
        model = gaussian_model.create_model()
        p_xy = model.marginal([gaussian_model.relative_x, gaussian_model.relative_y])
        fig = go.Figure(p_xy.root.plot(),p_xy.root.plotly_layout())
        # fig.show()


class FunkyPickUpTestCase(BulletWorldTestCase):

    def test_grounding(self):
        fpa = FunkyPickUpAction(ObjectDesignatorDescription(types=[ObjectType.MILK]).ground(), arms=["left", "right"],
                                grasps=["front", "left", "right", "top"])
        model = fpa.ground_model()
        self.assertIsInstance(model, ProbabilisticCircuit)


class RLTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object
    viz_marker_publisher: VizMarkerPublisher
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.viz_marker_publisher = VizMarkerPublisher()
        ProcessModule.execution_delay = False
        cls.engine = sqlalchemy.create_engine(pycrorm_uri)

    def setUp(self):
        self.world.reset_bullet_world()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self):
        self.world.reset_bullet_world()
        pycram.task.reset_tree()
        pycram.orm.base.ProcessMetaData.reset()
        self.session.rollback()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        self.session.close()

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

    def test_funky_pick_up(self):
        milk_description = ObjectDesignatorDescription(types=[ObjectType.MILK]).ground()
        fpa = FunkyPickUpAction(milk_description,
                                arms=[Arms.LEFT.value, Arms.RIGHT.value, Arms.BOTH.value],
                                grasps=[Grasp.FRONT.value, Grasp.LEFT.value, Grasp.RIGHT.value, Grasp.TOP.value])
        fpa.sample_amount = 1000
        relative_x = Continuous("relative_x")
        relative_y = Continuous("relative_y")
        p_xy = fpa.policy.marginal([relative_x, relative_y])
        fig = go.Figure(p_xy.root.plot(), p_xy.root.plotly_layout())
        # fig.show()
        with simulated_robot:
            fpa.try_out_policy()

        pycram.orm.base.ProcessMetaData().description = "costmap_no_obstacles_test"

        pycram.task.task_tree.root.insert(self.session)
        samples = pd.read_sql_query(fpa.query_for_database(), self.engine)
        samples = samples.rename(columns={"anon_1": "relative_x", "anon_2": "relative_y"})

        variables = infer_variables_from_dataframe(samples, scale_continuous_types=False)
        model = JPT(variables, min_samples_leaf=0.1)
        model.fit(samples)
        model = model.probabilistic_circuit
        arm, grasp, relative_x, relative_y = model.variables

        p_xy = model.marginal([relative_x, relative_y])
        fig = go.Figure(p_xy.root.plot_2d(5000), p_xy.root.plotly_layout())
        fig.show()


if __name__ == '__main__':
    unittest.main()
