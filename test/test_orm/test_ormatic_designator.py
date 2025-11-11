import unittest
from copy import deepcopy

import numpy as np
import sqlalchemy.sql.elements
from pycram.datastructures.enums import TorsoState, ApproachDirection, Arms, VerticalAlignment, DetectionTechnique, \
    DetectionState, GripperState
from semantic_digital_twin.robots.pr2 import PR2
from sqlalchemy import create_engine, select, text
from sqlalchemy.orm import Session

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import Pose, PoseStamped
from pycram.designator import ObjectDesignatorDescription, NamedObject
from pycram.language import SequentialPlan
from pycram.orm.logging_hooks import insert
from pycram.orm.ormatic_interface import *
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, ParkArmsAction, \
    DetectActionDescription, PlaceActionDescription, TransportAction, ParkArmsActionDescription, \
    TransportActionDescription, LookAtActionDescription, NavigateActionDescription, \
    PickUpActionDescription, SetGripperActionDescription, OpenActionDescription, CloseActionDescription, NavigateAction
from pycram.testing import BulletWorldTestCase
from semantic_digital_twin.world import World


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
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            plan = SequentialPlan(Context.from_world(test_world),
                                  NavigateActionDescription(
                                      PoseStamped.from_list([2, 2.2, 0], [0, 0, 0, 1], test_world.root), True),
                                  MoveTorsoActionDescription(TorsoState.HIGH),
                                  PickUpActionDescription(NamedObject("milk.stl"), Arms.LEFT,
                                                          GraspDescription(ApproachDirection.FRONT,
                                                                           VerticalAlignment.NoAlignment, False)),
                                  PlaceActionDescription(NamedObject("milk.stl"), [PoseStamped
                                                         .from_list( [2.1, 2, 0.9], [0, 0, 0, 1], test_world.root)],
                                                         [Arms.LEFT]))
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
            all([r.execution_data.execution_start_pose is not None and r.execution_data.execution_end_pose is not None for r in
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
        action = SequentialPlan(self.context,
                                NavigateActionDescription(PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1],self.world.root), True))
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(ResolvedActionNodeMappingDAO)).all()
       # motion = self.session.scalars(select(MoveMotionDAO)).all()
        self.assertEqual(result[0].action, NavigateAction)
        self.assertTrue(result[0].start_time < result[0].end_time)
        # self.assertEqual(result[1].action.dtype, MoveMotion.__name__)

    def test_inheritance(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan(Context.from_world(test_world),
                NavigateActionDescription(PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1],test_world.root), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(test_world.get_body_by_name("milk.stl"), Arms.LEFT,
                                        GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment,
                                                         False)),
                NavigateActionDescription(PoseStamped.from_list( [1.3, 1, 0], [0, 0, 0, 1], test_world.root), True),
                PlaceActionDescription(test_world.get_body_by_name("milk.stl"),
                                       PoseStamped.from_list([2.0, 1.6, 1.], [0, 0, 0, 1], test_world.root),
                                       Arms.LEFT))
            sp.perform()
        insert(sp, self.session)
        result = self.session.scalars(select(ActionDescriptionDAO)).all()
        self.assertEqual(len(result), 6)

    def test_parkArmsAction(self):
        action = SequentialPlan(self.context, ParkArmsActionDescription(Arms.BOTH))
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(ParkArmsActionDAO)).all()
        self.assertEqual(len(result), 1)
        self.assertEqual(type(result[0]).original_class(), ParkArmsAction)

    def test_transportAction(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        action = SequentialPlan(Context.from_world(test_world), TransportActionDescription(test_world.get_body_by_name("milk.stl"),
                                            PoseStamped.from_list( [1.3, 0.9, 0.9], [0, 0, 0, 1], test_world.root), Arms.LEFT))
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
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan(Context.from_world(test_world),
                NavigateActionDescription(PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1],test_world.root), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(test_world.get_body_by_name("milk.stl"), Arms.LEFT,
                                        GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment,
                                                         False)),
                NavigateActionDescription(PoseStamped.from_list( [1.3, 1, 0.], [0, 0, 0, 1], test_world.root), True),
                PlaceActionDescription(test_world.get_body_by_name("milk.stl"),
                                       PoseStamped.from_list( [2.0, 1.6, 1.], [0, 0, 0, 1], test_world.root),
                                       Arms.LEFT))
            sp.perform()
        insert(sp, self.session)
        result = self.session.scalars(select(PickUpActionDAO)).first()

    @unittest.skip("frozen object dosen't work atm")
    def test_lookAt_and_detectAction(self):
        object_description = ObjectDesignatorDescription(types=[Milk])
        action = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                         state=DetectionState.START,
                                         object_designator=object_description,
                                         region=None)
        with simulated_robot:
            sp = SequentialPlan(self.context,
                ParkArmsActionDescription(Arms.BOTH),
                NavigateActionDescription(PoseStamped.from_list( [0, 1, 0], [0, 0, 0, 1], self.world.root), True),
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
        action = SequentialPlan(self.context, SetGripperActionDescription(Arms.LEFT, GripperState.OPEN))
        with simulated_robot:
            action.perform()
        insert(action, self.session)
        result = self.session.scalars(select(SetGripperActionDAO)).all()
        self.assertEqual(result[0].gripper, Arms.LEFT)
        self.assertEqual(result[0].motion, GripperState.OPEN)


    def test_open_and_closeAction(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan( Context.from_world(test_world),
                ParkArmsActionDescription(Arms.BOTH),
                NavigateActionDescription(PoseStamped.from_list( [1.81, 1.73, 0.0],
                                                                [0.0, 0.0, 0.594, 0.804],test_world.root), True),
                OpenActionDescription(test_world.get_body_by_name("handle_cab10_t"), arm=Arms.LEFT, grasping_prepose_distance=0.03),
                CloseActionDescription(test_world.get_body_by_name("handle_cab10_t"), arm=Arms.LEFT, grasping_prepose_distance=0.03))
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

    @unittest.skip("no detect")
    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        with simulated_robot:
            #self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
            #self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
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

    @unittest.skip("no frozen objects")
    def test_type_casting(self):
        action = PickUpActionDescription(self.world.get_body_by_name("milk.stl"),  Arms.LEFT,
                                         GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment,
                                                          False))
        with simulated_robot:
            sp = SequentialPlan(self.context,
                NavigateActionDescription(PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1], self.world.root), True),
                ParkArmsActionDescription(Arms.BOTH),
                action,
                NavigateActionDescription(PoseStamped.from_list( [1.3, 1, 0.], [0, 0, 0, 1],self.world.root), True),
                PlaceActionDescription(self.world.get_body_by_name("milk.stl"),
                                       PoseStamped.from_list( [2.0, 1.6, 0.9], [0, 0, 0, 1], self.world.root),
                                       Arms.LEFT))
            sp.perform()

        insert(sp, self.session)
        object_result = self.session.scalars(select(FrozenObjectMappingDAO)).all()
        self.assertEqual(len(object_result), 2)




class ExecDataTest(ORMaticBaseTestCaseMixin):

    def plan(self, test_world):
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            with simulated_robot:
                sp = SequentialPlan(Context.from_world(test_world),
                                    NavigateActionDescription(
                                        PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1], test_world.root), True),
                                    ParkArmsActionDescription(Arms.BOTH),
                                    PickUpActionDescription(test_world.get_body_by_name("milk.stl"), Arms.LEFT,
                                                            GraspDescription(ApproachDirection.FRONT,
                                                                             VerticalAlignment.NoAlignment,
                                                                             False)),
                                    NavigateActionDescription(
                                        PoseStamped.from_list([1.3, 1, 0], [0, 0, 0, 1], test_world.root), True),
                                    MoveTorsoActionDescription(TorsoState.HIGH),
                                    PlaceActionDescription(test_world.get_body_by_name("milk.stl"),
                                                           PoseStamped.from_list( [2.0, 1.6, 1.],
                                                                                 [0, 0, 0, 1], test_world.root),
                                                           Arms.LEFT))

            sp.perform()
        insert(sp, self.session)

    def test_exec_creation(self):
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root), True),)

        with simulated_robot:
            plan.perform()
        insert(plan, self.session)
        exec_data = self.session.scalars(select(ExecutionDataDAO)).all()
        self.assertIsNotNone(exec_data)

    def test_pose(self):
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(
                                  PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1], self.world.root), True), )

        with simulated_robot:
            plan.perform()
        insert(plan, self.session)
        exec_data = self.session.scalars(select(ExecutionDataDAO)).all()[0]
        self.assertIsNotNone(exec_data)
        self.assertListEqual([0, 0, 0], PoseStampedDAO.from_dao(exec_data.execution_start_pose).pose.position.to_list())
        self.assertListEqual([0.6, 0.4, 0], PoseStampedDAO.from_dao(exec_data.execution_end_pose).pose.position.to_list())

    def test_manipulated_body_pose(self):
        test_world = deepcopy(self.world)

        self.plan(test_world)

        pick_up = self.session.scalars(select(PickUpActionDAO)).all()[0]
        place = self.session.scalars(select(PlaceActionDAO)).all()[0]
        self.assertIsNotNone(pick_up.execution_data.manipulated_body_pose_start)
        self.assertIsNotNone(pick_up.execution_data.manipulated_body_pose_end)
        start_pose_pick = PoseStampedDAO.from_dao(pick_up.execution_data.manipulated_body_pose_start)
        end_pose_pick = PoseStampedDAO.from_dao(pick_up.execution_data.manipulated_body_pose_end)
        start_pose_place = PoseStampedDAO.from_dao(place.execution_data.manipulated_body_pose_start)
        end_pose_place = PoseStampedDAO.from_dao(place.execution_data.manipulated_body_pose_end)

        self.assertListEqual([2.2, 2, 1], start_pose_pick.position.to_list())
        # Check that the end_pose of pick_up and start pose of place are not equal because of navigate in between
        for pick, place in zip(end_pose_pick.position.to_list(), start_pose_place.position.to_list()):
            self.assertNotEqual(pick, place)
        np.testing.assert_almost_equal([2.0, 1.6, 1], end_pose_place.position.to_list())

    def test_manipulated_body(self):
        test_world = deepcopy(self.world)

        self.plan(test_world)

        pick_up = self.session.scalars(select(PickUpActionDAO)).all()[0]
        self.assertIsNotNone(pick_up.execution_data.manipulated_body)
        milk = BodyDAO.from_dao(pick_up.execution_data.manipulated_body)
        # self.assertEqual(milk.name.name, "milk.stl")

    def test_state(self):
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(
                                  PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1],self.world.root), True), )
        with simulated_robot:
            plan.perform()
        insert(plan, self.session)
        navigate = self.session.scalars(select(NavigateActionDAO)).all()[0]
        self.assertIsNotNone(navigate.execution_data.execution_start_world_state)



class RelationalAlgebraTestCase(ORMaticBaseTestCaseMixin):
    def test_filtering(self):
        with simulated_robot:
            sp = SequentialPlan(self.context,
                NavigateActionDescription(PoseStamped.from_list( [0.6, 0.4, 0], [0, 0, 0, 1], self.world.root), True),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(self.world.get_body_by_name("milk.stl"), Arms.LEFT,
                                        GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment,
                                                         False)),
                NavigateActionDescription(PoseStamped.from_list( [1.3, 1, 0.], [0, 0, 0, 1], self.world.root), True),
                PlaceActionDescription(self.world.get_body_by_name("milk.stl"),
                                       PoseStamped.from_list( [2.0, 1.6, 0.9], [0, 0, 0, 1], self.world.root),
                                       Arms.LEFT))
            sp.perform()
        insert(sp, self.session)

        filtered_navigate_results = self.session.scalars(
            select(NavigateActionDAO).where(NavigateActionDAO.id == 1)).all()
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
#         torso_joint = RobotDescription.current_robot_description.torso_joint
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
#         torso_joint = RobotDescription.current_robot_description.torso_joint
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
