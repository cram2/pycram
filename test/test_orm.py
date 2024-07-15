import os
import unittest
from sqlalchemy import select
import sqlalchemy.orm
import pycram.orm.action_designator
import pycram.orm.base
import pycram.orm.motion_designator
import pycram.orm.object_designator
import pycram.orm.tasktree
import pycram.tasktree
from bullet_world_testcase import BulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycram.designators import action_designator, object_designator, motion_designator
from pycram.designators.action_designator import ParkArmsActionPerformable, MoveTorsoActionPerformable, \
    SetGripperActionPerformable, PickUpActionPerformable, NavigateActionPerformable, TransportActionPerformable, \
    OpenActionPerformable, CloseActionPerformable, DetectActionPerformable, LookAtActionPerformable
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.tasktree import with_tree
from pycram.orm.views import PickUpWithContextView
from pycram.datastructures.enums import Arms, Grasp, GripperState


class DatabaseTestCaseMixin(BulletWorldTestCase):
    engine: sqlalchemy.engine
    session: sqlalchemy.orm.Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)

    def setUp(self):
        super().setUp()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)

    def tearDown(self):
        super().tearDown()
        pycram.tasktree.reset_tree()
        pycram.orm.base.ProcessMetaData.reset()
        pycram.orm.base.Base.metadata.drop_all(self.engine)
        self.session.close()


class ORMTestSchemaTestCase(DatabaseTestCaseMixin, unittest.TestCase):
    def test_schema_creation(self):
        tables = list(pycram.orm.base.Base.metadata.tables.keys())
        self.assertTrue("Position" in tables)
        self.assertTrue("Quaternion" in tables)
        self.assertTrue("TaskTreeNode" in tables)
        self.assertTrue("Action" in tables)
        self.assertTrue("ParkArmsAction" in tables)
        self.assertTrue("NavigateAction" in tables)
        self.assertTrue("MoveTorsoAction" in tables)
        self.assertTrue("SetGripperAction" in tables)
        self.assertTrue("Release" in tables)
        self.assertTrue("GripAction" in tables)
        self.assertTrue("PickUpAction" in tables)
        self.assertTrue("PlaceAction" in tables)
        self.assertTrue("TransportAction" in tables)
        self.assertTrue("LookAtAction" in tables)
        self.assertTrue("DetectAction" in tables)
        self.assertTrue("OpenAction" in tables)
        self.assertTrue("CloseAction" in tables)


class ORMTaskTreeTestCase(DatabaseTestCaseMixin):

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()

    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "Unittest"
        pycram.tasktree.task_tree.root.insert(self.session, )

        node_results = self.session.scalars(select(pycram.orm.tasktree.TaskTreeNode)).all()
        self.assertEqual(len(node_results), len(pycram.tasktree.task_tree.root))

        position_results = self.session.scalars(select(pycram.orm.base.Position)).all()
        self.assertEqual(14, len(position_results))

        quaternion_results = self.session.scalars(select(pycram.orm.base.Quaternion)).all()
        self.assertEqual(14, len(quaternion_results))

        park_arms_results = self.session.scalars(select(pycram.orm.action_designator.ParkArmsAction)).all()
        self.assertEqual(0, len(park_arms_results))

        navigate_results = self.session.scalars(select(pycram.orm.action_designator.NavigateAction)).all()
        self.assertEqual(1, len(navigate_results))

        action_results = self.session.scalars(select(pycram.orm.action_designator.Action)).all()
        self.assertEqual(4, len(action_results))

    def test_metadata_existence(self):
        pycram.orm.base.ProcessMetaData().description = "metadata_existence_test"
        self.plan()
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.base.Pose)).all()
        self.assertTrue(all([r.process_metadata is not None for r in result]))

    def test_task_tree_node_parents(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "task_tree_node_parents_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.tasktree.TaskTreeNode)).all()
        self.assertTrue([result[i].parent == result[result[i].parent_id - 1] for i in range(len(result))
                         if result[i].parent_id is not None])

    def test_meta_data(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "Unittest"
        pycram.tasktree.task_tree.root.insert(self.session, )
        metadata_results = self.session.scalars(select(pycram.orm.base.ProcessMetaData)).all()
        self.assertEqual(1, len(metadata_results))

        action_results = self.session.scalars(select(pycram.orm.action_designator.Action)).all()
        self.assertTrue(all([a.process_metadata_id for a in action_results]))

        park_arms_results = self.session.scalars(select(pycram.orm.action_designator.ParkArmsAction)).all()
        self.assertTrue(all([a.process_metadata_id for a in park_arms_results]))

        object_results = self.session.scalars(select(pycram.orm.object_designator.Object)).all()
        self.assertTrue(all([o.process_metadata_id for o in object_results]))

    def test_meta_data_alternation(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "meta_data_alternation_test"
        pycram.tasktree.task_tree.root.insert(self.session, )
        metadata_result = self.session.scalars(select(pycram.orm.base.ProcessMetaData)).first()
        self.assertEqual(metadata_result.description, "meta_data_alternation_test")


class MixinTestCase(DatabaseTestCaseMixin):
    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()

    def test_pose(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "pose_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.base.Pose)).all()
        self.assertTrue(all([r.position is not None and r.orientation is not None for r in result]))

    def test_pose_mixin(self):
        self.plan()
        pycram.orm.base.ProcessMetaData().description = "pose_mixin_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.base.RobotState)).all()
        self.assertTrue(all([r.pose is not None and r.pose_id == r.pose.id for r in result]))


class ORMObjectDesignatorTestCase(DatabaseTestCaseMixin):
    """Test ORM functionality with a plan including object designators. """

    def test_plan_serialization(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()
        pycram.orm.base.ProcessMetaData().description = "Unittest"
        tt = pycram.tasktree.task_tree
        tt.insert(self.session)
        action_results = self.session.scalars(select(pycram.orm.action_designator.Action)).all()
        motion_results = self.session.scalars(select(pycram.orm.motion_designator.Motion)).all()
        self.assertEqual(len(tt) - 1, len(action_results) + len(motion_results))


class ORMActionDesignatorTestCase(DatabaseTestCaseMixin):

    def test_code_designator_type(self):
        action = NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1]))
        with simulated_robot:
            action.perform()
        pycram.orm.base.ProcessMetaData().description = "code_designator_type_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.tasktree.TaskTreeNode).where(pycram.orm.tasktree.TaskTreeNode.
                                                                                 action_id.isnot(None))).all()
        self.assertEqual(result[0].action.dtype, action_designator.NavigateAction.__name__)
        self.assertEqual(result[1].action.dtype, motion_designator.MoveMotion.__name__)

    def test_parkArmsAction(self):
        action = ParkArmsActionPerformable(pycram.datastructures.enums.Arms.BOTH)
        with simulated_robot:
            action.perform()
        pycram.orm.base.ProcessMetaData().description = "parkArmsAction_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.action_designator.ParkArmsAction)).all()
        self.assertTrue(all([result[i + 1].dtype is not pycram.orm.action_designator.Action.dtype
                             if result[i].dtype is pycram.orm.action_designator.ParkArmsAction.dtype else None
                             for i in range(len(result) - 1)]))

    def test_transportAction(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        action = TransportActionPerformable(object_description.resolve(), Arms.LEFT,
                                            Pose([1.3, 0.9, 0.9], [0, 0, 0, 1]))
        with simulated_robot:
            action.perform()
        pycram.orm.base.ProcessMetaData().description = "transportAction_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.action_designator.TransportAction)).all()
        milk_object = self.session.scalars(select(pycram.orm.object_designator.Object)).first()
        self.assertEqual(milk_object.pose, result[0].object.pose)

    def test_lookAt_and_detectAction(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        action = DetectActionPerformable(object_description.resolve())
        with simulated_robot:
            ParkArmsActionPerformable(pycram.datastructures.enums.Arms.BOTH).perform()
            NavigateActionPerformable(Pose([0, 1, 0], [0, 0, 0, 1])).perform()
            LookAtActionPerformable(object_description.resolve().pose).perform()
            action.perform()
        pycram.orm.base.ProcessMetaData().description = "detectAction_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.action_designator.DetectAction)).all()
        self.assertEqual(result[0].object.name, "milk")

    def test_setGripperAction(self):
        action = SetGripperActionPerformable(Arms.LEFT, GripperState.OPEN)
        with simulated_robot:
            action.perform()
        pycram.orm.base.ProcessMetaData().description = "setGripperAction_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(pycram.orm.action_designator.SetGripperAction)).all()
        self.assertEqual(result[0].gripper, Arms.LEFT)
        self.assertEqual(result[0].motion, GripperState.OPEN)

    def test_open_and_closeAction(self):
        apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
        apartment_desig = BelieveObject(names=["apartment"]).resolve()
        handle_desig = object_designator.ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig).resolve()

        self.kitchen.set_pose(Pose([20, 20, 0], [0, 0, 0, 1]))

        with simulated_robot:
            ParkArmsActionPerformable(pycram.datastructures.enums.Arms.BOTH).perform()
            NavigateActionPerformable(Pose([1.81, 1.73, 0.0],
                                           [0.0, 0.0, 0.594, 0.804])).perform()
            OpenActionPerformable(handle_desig, arm=Arms.LEFT).perform()
            CloseActionPerformable(handle_desig, arm=Arms.LEFT).perform()

        pycram.orm.base.ProcessMetaData().description = "open_and_closeAction_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        open_result = self.session.scalars(select(pycram.orm.action_designator.OpenAction)).all()
        close_result = self.session.scalars(select(pycram.orm.action_designator.CloseAction)).all()
        self.assertTrue(open_result is not None)
        self.assertEqual(open_result[0].object.name, "handle_cab10_t")
        self.assertTrue(close_result is not None)
        self.assertEqual(close_result[0].object.name, "handle_cab10_t")
        apartment.remove()


class ViewsSchemaTest(DatabaseTestCaseMixin):
    def test_view_creation(self):
        pycram.orm.base.ProcessMetaData().description = "view_creation_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        view = PickUpWithContextView
        self.assertEqual(len(view.__table__.columns), 12)
        self.assertEqual(view.__table__.name, "PickUpWithContextView")
        self.assertEqual(view.__table__.columns[0].name, "id")
        self.assertEqual(view.__table__.columns[1].name, "arm")
        self.assertEqual(view.__table__.columns[2].name, "grasp")
        self.assertEqual(view.__table__.columns[3].name, "torso_height")
        self.assertEqual(view.__table__.columns[4].name, "relative_x")
        self.assertEqual(view.__table__.columns[5].name, "relative_y")
        self.assertEqual(view.__table__.columns[6].name, "quaternion_x")
        self.assertEqual(view.__table__.columns[7].name, "quaternion_y")
        self.assertEqual(view.__table__.columns[8].name, "quaternion_z")
        self.assertEqual(view.__table__.columns[9].name, "quaternion_w")
        self.assertEqual(view.__table__.columns[10].name, "obj_type")
        self.assertEqual(view.__table__.columns[11].name, "status")

    def test_pickUpWithContextView(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()
        pycram.orm.base.ProcessMetaData().description = "pickUpWithContextView_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(PickUpWithContextView)).first()
        self.assertEqual(result.arm, Arms.LEFT)
        self.assertEqual(result.grasp, Grasp.FRONT)
        self.assertEqual(result.torso_height, 0.3)
        self.assertAlmostEqual(result.relative_x, -0.7, 6)
        self.assertAlmostEqual(result.relative_y, -0.6, 6)
        self.assertEqual(result.quaternion_x, 0)
        self.assertEqual(result.quaternion_w, 1)

    def test_pickUpWithContextView_conditions(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()
        pycram.orm.base.ProcessMetaData().description = "pickUpWithContextView_conditions_test"
        pycram.tasktree.task_tree.root.insert(self.session)
        result = self.session.scalars(select(PickUpWithContextView)
                                      .where(PickUpWithContextView.arm == Arms.RIGHT)).all()
        self.assertEqual(result, [])


if __name__ == '__main__':
    unittest.main()
