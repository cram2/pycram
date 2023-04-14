import unittest

import sqlalchemy
import sqlalchemy.orm

import pycram.orm.action_designator
import pycram.orm.base
import pycram.orm.motion_designator
import pycram.orm.object_designator
import pycram.orm.task
import pycram.task
import pycram.task
import test_bullet_world
import test_task_tree
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.resolver.plans import Arms
from pycram.task import with_tree
import anytree


class ORMTestSchema(unittest.TestCase):

    def test_schema_creation(self):
        self.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session.commit()
        tables = list(pycram.orm.base.Base.metadata.tables.keys())
        self.assertTrue("Position" in tables)
        self.assertTrue("Quaternion" in tables)
        self.assertTrue("TaskTreeNode" in tables)
        self.assertTrue("Code" in tables)
        self.assertTrue("Action" in tables)
        self.assertTrue("ParkArms" in tables)
        self.assertTrue("Navigate" in tables)
        self.assertTrue("MoveTorso" in tables)
        self.assertTrue("SetGripper" in tables)
        self.assertTrue("Release" in tables)
        self.assertTrue("Grip" in tables)
        self.assertTrue("PickUp" in tables)
        self.assertTrue("Place" in tables)
        self.assertTrue("Transport" in tables)
        self.assertTrue("LookAt" in tables)
        self.assertTrue("Detect" in tables)
        self.assertTrue("Open" in tables)
        self.assertTrue("Close" in tables)


class ORMTaskTreeTestCase(test_task_tree.TaskTreeTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)

    def setUp(self):
        super().setUp()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self):
        super().tearDown()
        self.session.close()

    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        self.plan()
        pycram.task.task_tree.root.insert(self.session)

        node_results = self.session.query(pycram.orm.task.TaskTreeNode).all()
        self.assertEqual(len(node_results), len(pycram.task.task_tree.root))

        code_results = self.session.query(pycram.orm.task.Code).all()
        self.assertEqual(len(code_results), len(pycram.task.task_tree.root))

        position_results = self.session.query(pycram.orm.base.Position).all()
        self.assertEqual(len(position_results), 1)

        quaternion_results = self.session.query(pycram.orm.base.Quaternion).all()
        self.assertEqual(len(quaternion_results), 1)

        park_arms_results = self.session.query(pycram.orm.action_designator.ParkArmsAction).all()
        self.assertEqual(len(park_arms_results), 2)

        navigate_results = self.session.query(pycram.orm.action_designator.NavigateAction).all()
        self.assertEqual(len(navigate_results), 1)

        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertEqual(len(action_results), 3)

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()
        pycram.orm.base.Base.metadata.drop_all(cls.engine)
        cls.session.commit()


class ORMObjectDesignatorTestCase(test_bullet_world.BulletWorldTest):
    """Test ORM functionality with a plan including object designators. """

    @with_tree
    def plan(self):
        with simulated_robot:
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()
            ActionDesignator(MoveTorsoAction(0.2)).perform()
            location = LocationDesignator(CostmapLocation(target=self.milk, reachable_for=self.robot))
            pose = location.reference()
            ActionDesignator(
                NavigateAction(target_position=pose["position"], target_orientation=pose["orientation"])).perform()
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

            picked_up_arm = pose["arms"][0]
            ActionDesignator(
                PickUpAction(object_designator=self.milk_desig, arm=pose["arms"][0], grasp="front")).perform()

            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()
            place_island = LocationDesignator(
                SemanticCostmapLocation("kitchen_island_surface", self.kitchen, self.milk_desig.prop_value("object")))
            pose_island = place_island.reference()

            place_location = LocationDesignator(
                CostmapLocation(target=list(pose_island.values()), reachable_for=self.robot,
                                reachable_arm=picked_up_arm))
            pose = place_location.reference()

            ActionDesignator(
                NavigateAction(target_position=pose["position"], target_orientation=pose["orientation"])).perform()

            ActionDesignator(PlaceAction(object_designator=self.milk_desig, target_location=list(pose_island.values()),
                                         arm=picked_up_arm)).perform()

            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        cls.milk_desig = ObjectDesignator(ObjectDesignatorDescription(name="milk", type="milk"))
        cls.cereal_desig = ObjectDesignator(ObjectDesignatorDescription(name="cereal", type="cereal"))
        cls.session = sqlalchemy.orm.Session(bind=cls.engine)

    def setUp(self):
        super().setUp()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session.commit()

    def tearDown(self):
        super().tearDown()
        pycram.task.reset_tree()

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()
        pycram.orm.base.Base.metadata.drop_all(cls.engine)
        cls.session.commit()
        cls.session.close()

    def test_plan_serialization(self):
        self.plan()
        tt = pycram.task.task_tree
        print(anytree.RenderTree(tt))
        tt.insert(self.session)
        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertEqual(len(tt) - 2, len(action_results))


if __name__ == '__main__':
    unittest.main()
