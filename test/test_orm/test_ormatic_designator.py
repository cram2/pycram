import unittest

import sqlalchemy.sql.elements
from sqlalchemy import create_engine, select, text
from sqlalchemy.orm import registry, Session

from pycram.datastructures.dataclasses import FrozenObject, Color
from pycram.datastructures.enums import TorsoState, Arms, Grasp, DetectionTechnique, DetectionState, GripperState, \
    WorldMode, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import Vector3, Pose, PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.multirobot import RobotManager
from pycram.robot_plans import MoveTorsoActionDescription, ParkArmsAction, \
    MoveTorsoAction, NavigateAction, PickUpAction, DetectActionDescription, DetectAction, \
    PlaceActionDescription, TransportAction, PlaceAction, LookAtAction, SetGripperAction, OpenAction, CloseAction, \
    ParkArmsActionDescription, TransportActionDescription, LookAtActionDescription, NavigateActionDescription, \
    PickUpActionDescription, SetGripperActionDescription, OpenActionDescription, CloseActionDescription
from pycram.designators.object_designator import BelieveObject, ObjectPart
from pycram.language import SequentialPlan
from pycram.orm.logging_hooks import insert
from pycram.orm.ormatic_interface import *
from pycram.plan import ResolvedActionNode
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescription
from pycram.testing import BulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Milk, Apartment, Robot


class ORMaticBaseTestCaseMixin(BulletWorldTestCase):
    engine: sqlalchemy.engine
    session: Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = create_engine('sqlite:///:memory:')

    def setUp(self):
        super().setUp()
        self.session = Session(self.engine)
        Base.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        Base.metadata.drop_all(self.session.bind)
        self.session.close()


class PoseTestCases(ORMaticBaseTestCaseMixin):

    def plan(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = PlaceActionDescription(object_description, [PoseStamped
                                             .from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        torso_joint = RobotManager.get_robot_description().torso_joint
        self.assertEqual(description.resolve().object_designator.name, "milk")
        with simulated_robot:
            plan = SequentialPlan(
                NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                MoveTorsoActionDescription(TorsoState.HIGH),
                PickUpActionDescription(object_description.resolve(), Arms.LEFT, GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)),
                description)
            plan.perform()
        return plan

    def test_pose(self):
        plan = self.plan()
        insert(plan, self.session)
        result = self.session.scalars(select(PoseDAO)).all()
        self.assertGreater(len(result), 0)
        self.assertTrue(all([r.position is not None and r.orientation is not None for r in result]))

    def test_action_to_pose(self):
        plan = self.plan()
        insert(plan, self.session)
        result = self.session.scalars(select(ActionDescriptionDAO)).all()
        self.assertTrue(
            all([r.robot_position.pose is not None and r.robot_position.pose_id == r.robot_position.pose.id for r in
                 result]))

    def test_pose_vs_pose_stamped(self):
        plan = self.plan()
        insert(plan, self.session)
        pose_stamped_result = self.session.scalars(select(PoseStampedDAO)).all()
        pose_result = self.session.scalars(select(PoseDAO)).all()
        poses_from_pose_stamped_results = (self.session
                                           .scalars(select(PoseDAO)
                                                    .where(
            PoseDAO.id.in_([r.pose_id for r in pose_stamped_result]))).all())
        self.assertTrue(all([r.pose is not None for r in pose_stamped_result]))
        self.assertTrue(all([r.position is not None and r.orientation is not None for r in pose_result]))
        self.assertEqual(len(poses_from_pose_stamped_results), len(pose_result))
        self.assertEqual(pose_stamped_result[0].pose_id, pose_result[0].id)

    def test_pose_creation(self):
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 3.0
        pose.orientation.x = 4.0
        pose.orientation.y = 5.0
        pose.orientation.z = 6.0
        pose.orientation.w = 7.0

        pose_dao = PoseDAO.to_dao(pose)

        self.session.add(pose_dao.position)
        self.session.add(pose_dao.orientation)
        self.session.add(pose_dao)
        self.session.commit()

        with self.session.bind.connect() as conn:
            raw_pose = conn.execute(text("SELECT * FROM PoseDAO")).fetchall()

        pose_result = self.session.scalars(select(PoseDAO)).first()
        self.assertEqual(pose_result.position.x, 1.0)
        self.assertEqual(pose_result.position.y, 2.0)
        self.assertEqual(pose_result.position.z, 3.0)
        self.assertEqual(pose_result.id, raw_pose[0][0])


class ORMActionDesignatorTestCase(ORMaticBaseTestCaseMixin):
    def test_code_designator_type(self):
        action = NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True)
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(ResolvedActionNodeMappingDAO)).all()
        # self.assertEqual(result[0].action, NavigateAction)
        # self.assertEqual(result[1].action.dtype, MoveMotion.__name__)

    def test_inheritance(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        with simulated_robot:
            sp = SequentialPlan(
                NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(object_description.resolve(), Arms.LEFT,
                                        GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)),
                NavigateActionDescription(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True),
                PlaceActionDescription(object_description.resolve(),
                                       PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                                       Arms.LEFT))
            sp.perform()
        insert(sp, self.session)
        result = self.session.scalars(select(ActionDescriptionDAO)).all()
        self.assertEqual(len(result), 5)

    def test_parkArmsAction(self):
        action = ParkArmsActionDescription(Arms.BOTH)
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(ParkArmsActionDAO)).all()
        self.assertEqual(len(result), 1)
        self.assertEqual(type(result[0]).original_class(), ParkArmsAction)

    def test_transportAction(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        action = TransportActionDescription(object_description.resolve(),
                                 PoseStamped.from_list([1.3, 0.9, 0.9], [0, 0, 0, 1]), Arms.LEFT)
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(TransportActionDAO)).all()

        self.assertEqual(type(result[0]), TransportActionDAO)
        self.assertEqual(result[0].original_class(), TransportAction)
        self.assertTrue(result[0].target_location is not None)
        result = self.session.scalars(select(TransportActionDAO)).first()
        self.assertIsNotNone(result)

    def test_pickUpAction(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        with simulated_robot:
            sp = SequentialPlan(
                NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(object_description.resolve(), Arms.LEFT, GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)),
                NavigateActionDescription(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True),
                PlaceActionDescription(object_description.resolve(), PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                            Arms.LEFT))
            sp.perform()
        insert(sp, self.session)
        result = self.session.scalars(select(PickUpActionDAO)).first()
        frozen_objects = self.session.scalars(select(FrozenObjectMappingDAO)).all()
        self.assertIsNotNone(result.object_at_execution)

    @unittest.skip("frozen object dosen't work atm")
    def test_lookAt_and_detectAction(self):
        object_description = ObjectDesignatorDescription(types=[Milk])
        action = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                         state=DetectionState.START,
                                         object_designator=object_description,
                                         region=None)
        with simulated_robot:
            sp = SequentialPlan(
                ParkArmsActionDescription(Arms.BOTH),
                NavigateActionDescription(PoseStamped.from_list([0, 1, 0], [0, 0, 0, 1]), True),
                LookAtActionDescription(object_description.resolve().pose),
                action)
            sp.perform()
        insert(sp, self.session)
        object_result = self.session.scalars(select(FrozenObjectMappingDAO)).all()
        result = self.session.scalars(select(DetectActionDAO)).all()
        self.assertEqual(len(object_result), 1)
        self.assertEqual(object_result[0].name, "milk")
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].technique, DetectionTechnique.TYPES)
        self.assertEqual(result[0].state, DetectionState.START)
        self.assertEqual(result[0].object_at_execution.name, "milk")

    def test_setGripperAction(self):
        action = SetGripperActionDescription(Arms.LEFT, GripperState.OPEN)
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(SetGripperActionDAO)).all()
        self.assertEqual(result[0].gripper, Arms.LEFT)
        self.assertEqual(result[0].motion, GripperState.OPEN)

    @unittest.skip
    # TODO fix Pose for OpenAction
    def test_open_and_closeAction(self):
        apartment = Object("apartment", Apartment, "apartment.urdf")
        apartment_desig = BelieveObject(names=["apartment"]).resolve()
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig).resolve()

        self.kitchen.set_pose(PoseStamped.from_list([20, 20, 0], [0, 0, 0, 1]))

        with simulated_robot:
            sp = SequentialPlan(
                ParkArmsActionDescription(Arms.BOTH),
                NavigateActionDescription(PoseStamped.from_list([1.81, 1.73, 0.0],
                                                     [0.0, 0.0, 0.594, 0.804]), True),
                OpenActionDescription(handle_desig, arm=Arms.LEFT, grasping_prepose_distance=0.03),
                CloseActionDescription(handle_desig, arm=Arms.LEFT, grasping_prepose_distance=0.03))
            sp.perform()
        insert(sp, self.session)
        open_result = self.session.scalars(select(OpenActionDAO)).all()
        close_result = self.session.scalars(select(CloseActionDAO)).all()
        self.assertTrue(open_result is not None)
        # can not do that yet with new mapping
        # self.assertEqual(open_result[0].object.name, "handle_cab10_t")
        self.assertTrue(close_result is not None)
        # can not do that yet with new mapping
        # self.assertEqual(close_result[0].object.name, "handle_cab10_t")
        apartment.remove()

    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        with simulated_robot:
            self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
            self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
            # object_description = ObjectDesignatorDescription(types=[Milk])
            # pr2 = Object("pr2", Robot, "pr2.urdf", pose=PoseStamped.from_list([1, 2, 0]))
            description = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                                  object_designator=self.milk)
            description.perform()

        insert(description, self.session)
        # node_results = self.session.scalars(select(TaskTreeNodeORM)).all()
        # self.assertEqual(len(node_results), len(description.root))

        action_results = self.session.scalars(select(ActionDescriptionDAO)).all()
        self.assertEqual(1, len(action_results))

        detect_actions = self.session.scalars(select(DetectActionDAO)).all()
        self.assertEqual(1, len(detect_actions))

    def test_type_casting(self):
        object_description = ObjectDesignatorDescription(names=["milk"], types=[Milk])
        action = PickUpActionDescription(object_description.resolve(), Arms.LEFT,
                                GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False))
        with simulated_robot:
            sp = SequentialPlan(
                NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                ParkArmsActionDescription(Arms.BOTH),
                action,
                NavigateActionDescription(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True),
                PlaceActionDescription(object_description.resolve(),
                                       PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                                       Arms.LEFT))
            sp.perform()

        insert(sp, self.session)
        object_result = self.session.scalars(select(FrozenObjectMappingDAO)).all()
        self.assertEqual(len(object_result), 2)


class RelationalAlgebraTestCase(ORMaticBaseTestCaseMixin):
    def test_filtering(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        with simulated_robot:
            sp = SequentialPlan(
                NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(object_description.resolve(), Arms.LEFT, GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)),
                NavigateActionDescription(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True),
                PlaceActionDescription(object_description.resolve(), PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                                       Arms.LEFT))
            sp.perform()
        insert(sp, self.session)

        filtered_navigate_results = self.session.scalars(select(NavigateActionDAO).where(NavigateActionDAO.id == 1)).all()
        self.assertEqual(1, len(filtered_navigate_results))


# Tests for when views are fixed

# class ViewsSchemaTest(DatabaseTestCaseMixin):
#
#     def test_view_creation(self):
#         pycram.orm.base.ProcessMetaData().description = "view_creation_test"
#         pycram.tasktree.task_tree.root.insert(self.session)
#         view = PickUpWithContextView
#         self.assertEqual(len(view.__table__.columns), 14)
#         self.assertEqual(view.__table__.name, "PickUpWithContextView")
#         self.assertEqual(view.__table__.columns[0].name, "id")
#         self.assertEqual(view.__table__.columns[1].name, "arm")
#         self.assertEqual(view.__table__.columns[2].name, "approach_direction")
#         self.assertEqual(view.__table__.columns[3].name, "vertical_alignment")
#         self.assertEqual(view.__table__.columns[4].name, "rotate_gripper")
#         self.assertEqual(view.__table__.columns[5].name, "torso_height")
#         self.assertEqual(view.__table__.columns[6].name, "relative_x")
#         self.assertEqual(view.__table__.columns[7].name, "relative_y")
#         self.assertEqual(view.__table__.columns[8].name, "x")
#         self.assertEqual(view.__table__.columns[9].name, "y")
#         self.assertEqual(view.__table__.columns[10].name, "z")
#         self.assertEqual(view.__table__.columns[11].name, "w")
#         self.assertEqual(view.__table__.columns[12].name, "obj_type")
#         self.assertEqual(view.__table__.columns[13].name, "status")
#
#     def test_pickUpWithContextView(self):
#         if self.engine.dialect.name == "sqlite":
#             return
#         object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
#         description = action_designator.PlaceActionDescription(object_description, [PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
#         torso_joint = RobotManager.get_robot_description().torso_joint
#         self.assertEqual(description.resolve().object_designator.name, "milk")
#         with simulated_robot:
#             NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
#             MoveTorsoAction(TorsoState.HIGH).perform()
#             grasp = GraspDescription(Grasp.FRONT, None, False)
#             PickUpAction(object_description.resolve(), Arms.LEFT, grasp).perform()
#             description.resolve().perform()
#         pycram.orm.base.ProcessMetaData().description = "pickUpWithContextView_test"
#         pycram.tasktree.task_tree.root.insert(self.session)
#         result = self.session.scalars(select(PickUpWithContextView)).first()
#         self.assertEqual(result.arm, Arms.LEFT)
#         self.assertEqual(result.grasp, grasp)
#         self.assertEqual(result.torso_height, 0.3)
#         self.assertAlmostEqual(result.relative_x, -0.7, 6)
#         self.assertAlmostEqual(result.relative_y, -0.6, 6)
#         self.assertEqual(result.quaternion_x, 0)
#         self.assertEqual(result.quaternion_w, 1)
#
#     def test_pickUpWithContextView_conditions(self):
#         if self.engine.dialect.name == "sqlite":
#             return
#         object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
#         description = action_designator.PlaceActionDescription(object_description, [PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
#         torso_joint = RobotManager.get_robot_description().torso_joint
#         self.assertEqual(description.resolve().object_designator.name, "milk")
#         with simulated_robot:
#             NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
#             MoveTorsoAction(TorsoState.HIGH).perform()
#             grasp = GraspDescription(Grasp.FRONT, None, False)
#             PickUpAction(object_description.resolve(), Arms.LEFT, grasp).perform()
#             description.resolve().perform()
#         pycram.orm.base.ProcessMetaData().description = "pickUpWithContextView_conditions_test"
#         pycram.tasktree.task_tree.root.insert(self.session)
#         result = self.session.scalars(select(PickUpWithContextView)
#                                       .where(PickUpWithContextView.arm == Arms.RIGHT)).all()
#         self.assertEqual(result, [])
