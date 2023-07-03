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
from pycram.designators import action_designator, object_designator
from pycram.process_module import simulated_robot
from pycram.task import with_tree


class ORMTestSchema(unittest.TestCase):
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)

    def setUp(self):
        super().setUp()
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self):
        super().tearDown()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        self.session.close()

    @classmethod
    def TearDownClass(cls):
        super().tearDownClass()
        cls.session.commit()
        cls.session.close()

    def test_schema_creation(self):
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
    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

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
        pycram.task.reset_tree()
        pycram.orm.base.MetaData.reset()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        self.session.close()

    @classmethod
    def TearDownClass(cls):
        super().tearDownClass()
        cls.session.commit()
        cls.session.close()

    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        self.plan()
        pycram.orm.base.MetaData().description = "Unittest"
        pycram.task.task_tree.root.insert(self.session, )

        node_results = self.session.query(pycram.orm.task.TaskTreeNode).all()
        self.assertEqual(len(node_results), len(pycram.task.task_tree.root))

        code_results = self.session.query(pycram.orm.task.Code).all()
        self.assertEqual(len(code_results), len(pycram.task.task_tree.root))

        position_results = self.session.query(pycram.orm.base.Position).all()
        self.assertEqual(8, len(position_results))

        quaternion_results = self.session.query(pycram.orm.base.Quaternion).all()
        self.assertEqual(8, len(quaternion_results))

        park_arms_results = self.session.query(pycram.orm.action_designator.ParkArmsAction).all()
        self.assertEqual(0, len(park_arms_results))

        navigate_results = self.session.query(pycram.orm.action_designator.NavigateAction).all()
        self.assertEqual(1, len(navigate_results))

        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertEqual(4, len(action_results))

    def test_meta_data(self):
        self.plan()
        pycram.orm.base.MetaData().description = "Unittest"
        pycram.task.task_tree.root.insert(self.session, )
        metadata_results = self.session.query(pycram.orm.base.MetaData).all()
        self.assertEqual(1, len(metadata_results))

        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertTrue(all([a.metadata_id for a in action_results]))

        park_arms_results = self.session.query(pycram.orm.action_designator.ParkArmsAction).all()
        self.assertTrue(all([a.metadata_id for a in park_arms_results]))

        object_results = self.session.query(pycram.orm.object_designator.ObjectDesignator).all()
        self.assertTrue(all([o.metadata_id for o in object_results]))

    def test_meta_data_alternation(self):
        meta_data = pycram.orm.base.MetaData()
        meta_data.description = "Test"
        self.plan()
        pycram.task.task_tree.root.insert(self.session, )
        metadata_result = self.session.query(pycram.orm.base.MetaData).first()
        print(metadata_result)
        self.assertEqual(metadata_result.description, "Test")


class ORMObjectDesignatorTestCase(test_bullet_world.BulletWorldTest):
    """Test ORM functionality with a plan including object designators. """

    engine: sqlalchemy.engine.Engine
    session: sqlalchemy.orm.Session

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            action_designator.NavigateAction.Action(([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            action_designator.MoveTorsoAction.Action(0.3).perform()
            action_designator.PickUpAction.Action(object_description.resolve(), "left", "front").perform()
            description.resolve().perform()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        cls.session = sqlalchemy.orm.Session(bind=cls.engine)

    def setUp(self):
        super().setUp()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session.commit()

    def tearDown(self):
        super().tearDown()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        pycram.task.reset_tree()

    @classmethod
    def tearDownClass(cls):
        super().tearDownClass()
        cls.session.commit()
        cls.session.close()

    def test_plan_serialization(self):
        self.plan()
        pycram.orm.base.MetaData().description = "Unittest"
        tt = pycram.task.task_tree
        tt.insert(self.session)
        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertEqual(len(tt) - 2, len(action_results))


if __name__ == '__main__':
    unittest.main()
