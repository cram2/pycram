from copy import deepcopy

import numpy as np
import sqlalchemy.sql.elements

from krrood.entity_query_language.symbol_graph import SymbolGraph
from krrood.ormatic.dao import to_dao
from krrood.ormatic.utils import create_engine
from semantic_digital_twin.robots.pr2 import PR2
from sqlalchemy import select, text
from sqlalchemy.orm import Session

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    TorsoState,
    ApproachDirection,
    Arms,
    VerticalAlignment,
    GripperState,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import Pose, PoseStamped
from pycram.designator import NamedObject
from pycram.language import SequentialPlan
from pycram.orm.ormatic_interface import *
from pycram.process_module import simulated_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsAction,
    PlaceActionDescription,
    TransportAction,
    ParkArmsActionDescription,
    TransportActionDescription,
    NavigateActionDescription,
    PickUpActionDescription,
    SetGripperActionDescription,
    OpenActionDescription,
    CloseActionDescription,
    NavigateAction,
    PickUpAction,
    PlaceAction,
)
from pycram.testing import ApartmentWorldTestCase


class ORMaticBaseTestCaseMixin(ApartmentWorldTestCase):
    engine: sqlalchemy.engine
    session: Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = create_engine("sqlite:///:memory:")

    def setUp(self):
        super().setUp()
        self.session = Session(self.engine)
        Base.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        Base.metadata.drop_all(self.session.bind)
        self.session.expunge_all()
        self.session.close()


class PoseTestCases(ORMaticBaseTestCaseMixin):

    def plan(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            plan = SequentialPlan(
                Context.from_world(test_world),
                NavigateActionDescription(
                    PoseStamped.from_list([2, 2.2, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                MoveTorsoActionDescription(TorsoState.HIGH),
                PickUpActionDescription(
                    NamedObject("milk.stl"),
                    Arms.LEFT,
                    GraspDescription(
                        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                    ),
                ),
                PlaceActionDescription(
                    NamedObject("milk.stl"),
                    [
                        PoseStamped.from_list(
                            [2.1, 2, 0.9], [0, 0, 0, 1], test_world.root
                        )
                    ],
                    [Arms.LEFT],
                ),
            )
            plan.perform()
        return plan

    def test_pose(self):
        plan = self.plan()
        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        result = self.session.scalars(select(PoseDAO)).all()
        self.assertGreater(len(result), 0)
        self.assertTrue(
            all([r.position is not None and r.orientation is not None for r in result])
        )

    def test_action_to_pose(self):
        plan = self.plan()
        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        # result = self.session.scalars(select(ActionDescriptionDAO)).all()
        result = self.session.scalars(
            select(ResolvedActionNodeMappingDAO).where(
                ResolvedActionNodeMappingDAO.designator_type == NavigateAction
            )
        ).all()
        self.assertTrue(
            all(
                [
                    r.execution_data.execution_start_pose is not None
                    and r.execution_data.execution_end_pose is not None
                    for r in result
                ]
            )
        )

    def test_pose_vs_pose_stamped(self):
        plan = self.plan()
        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        pose_stamped_result = self.session.scalars(select(PoseStampedDAO)).all()
        pose_result = self.session.scalars(select(PoseDAO)).all()
        poses_from_pose_stamped_results = self.session.scalars(
            select(PoseDAO).where(
                PoseDAO.database_id.in_([r.pose_id for r in pose_stamped_result])
            )
        ).all()
        self.assertTrue(all([r.pose is not None for r in pose_stamped_result]))
        self.assertTrue(
            all(
                [
                    r.position is not None and r.orientation is not None
                    for r in pose_result
                ]
            )
        )
        self.assertEqual(len(poses_from_pose_stamped_results), len(pose_result))
        self.assertEqual(pose_stamped_result[0].pose_id, pose_result[0].database_id)

    def test_pose_creation(self):
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 3.0
        pose.orientation.x = 4.0
        pose.orientation.y = 5.0
        pose.orientation.z = 6.0
        pose.orientation.w = 7.0

        pose_dao = to_dao(pose)

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
        self.assertEqual(pose_result.database_id, raw_pose[0][0])


class ORMActionDesignatorTestCase(ORMaticBaseTestCaseMixin):
    def test_code_designator_type(self):
        action = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root),
                True,
            ),
        )
        with simulated_robot:
            action.perform()
        dao = to_dao(action)
        self.session.add(dao)
        self.session.commit()

        result = self.session.scalars(select(ResolvedActionNodeMappingDAO)).all()
        # motion = self.session.scalars(select(MoveMotionDAO)).all()
        self.assertEqual(result[0].designator_type, NavigateAction)
        self.assertTrue(result[0].start_time < result[0].end_time)
        # self.assertEqual(result[1].action.dtype, MoveMotion.__name__)

    def test_inheritance(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan(
                Context.from_world(test_world),
                NavigateActionDescription(
                    PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    Arms.LEFT,
                    GraspDescription(
                        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                    ),
                ),
                NavigateActionDescription(
                    PoseStamped.from_list([1.3, 1, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                PlaceActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    PoseStamped.from_list(
                        [2.0, 1.6, 1.0], [0, 0, 0, 1], test_world.root
                    ),
                    Arms.LEFT,
                ),
            )
            sp.perform()
        dao = to_dao(sp)
        self.session.add(dao)
        self.session.commit()

        result = self.session.scalars(select(ActionDescriptionDAO)).all()
        self.assertEqual(len(result), 6)

    def test_parkArmsAction(self):
        action = SequentialPlan(self.context, ParkArmsActionDescription(Arms.BOTH))
        with simulated_robot:
            action.perform()
        dao = to_dao(action)
        self.session.add(dao)
        self.session.commit()
        result = self.session.scalars(select(ParkArmsActionDAO)).all()
        self.assertEqual(1, len(result))
        self.assertEqual(type(result[0]).original_class(), ParkArmsAction)

    def test_transportAction(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        action = SequentialPlan(
            Context.from_world(test_world),
            TransportActionDescription(
                test_world.get_body_by_name("milk.stl"),
                PoseStamped.from_list([1.3, 0.9, 0.9], [0, 0, 0, 1], test_world.root),
                Arms.LEFT,
            ),
        )
        with simulated_robot:
            action.perform()
        dao = to_dao(action)
        self.session.add(dao)
        self.session.commit()
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
            sp = SequentialPlan(
                Context.from_world(test_world),
                NavigateActionDescription(
                    PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    Arms.LEFT,
                    GraspDescription(
                        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                    ),
                ),
                NavigateActionDescription(
                    PoseStamped.from_list([1.3, 1, 0.0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                PlaceActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    PoseStamped.from_list(
                        [2.0, 1.6, 1.0], [0, 0, 0, 1], test_world.root
                    ),
                    Arms.LEFT,
                ),
            )
            sp.perform()
        dao = to_dao(sp)
        self.session.add(dao)
        self.session.commit()
        result = self.session.scalars(select(PickUpActionDAO)).first()

    def test_setGripperAction(self):
        action = SequentialPlan(
            self.context, SetGripperActionDescription(Arms.LEFT, GripperState.OPEN)
        )
        with simulated_robot:
            action.perform()
        dao = to_dao(action)
        self.session.add(dao)
        self.session.commit()
        result = self.session.scalars(select(SetGripperActionDAO)).all()
        self.assertEqual(result[0].gripper, Arms.LEFT)
        self.assertEqual(result[0].motion, GripperState.OPEN)

    def test_open_and_closeAction(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan(
                Context.from_world(test_world),
                ParkArmsActionDescription(Arms.BOTH),
                NavigateActionDescription(
                    PoseStamped.from_list(
                        [1.81, 1.73, 0.0], [0.0, 0.0, 0.594, 0.804], test_world.root
                    ),
                    True,
                ),
                OpenActionDescription(
                    test_world.get_body_by_name("handle_cab10_t"),
                    arm=Arms.LEFT,
                    grasping_prepose_distance=0.03,
                ),
                CloseActionDescription(
                    test_world.get_body_by_name("handle_cab10_t"),
                    arm=Arms.LEFT,
                    grasping_prepose_distance=0.03,
                ),
            )
            sp.perform()
        dao = to_dao(sp)
        self.session.add(dao)
        self.session.commit()
        open_result = self.session.scalars(select(OpenActionDAO)).all()
        close_result = self.session.scalars(select(CloseActionDAO)).all()
        self.assertTrue(open_result is not None)
        # can not do that yet with new mapping
        # self.assertEqual(open_result[0].object.name, "handle_cab10_t")
        self.assertTrue(close_result is not None)
        # can not do that yet with new mapping
        # self.assertEqual(close_result[0].object.name, "handle_cab10_t")


class ExecDataTest(ORMaticBaseTestCaseMixin):

    def plan(self, test_world):
        test_robot = PR2.from_world(test_world)
        with simulated_robot:
            sp = SequentialPlan(
                Context.from_world(test_world),
                NavigateActionDescription(
                    PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    Arms.LEFT,
                    GraspDescription(
                        ApproachDirection.FRONT,
                        VerticalAlignment.NoAlignment,
                        False,
                    ),
                ),
                NavigateActionDescription(
                    PoseStamped.from_list([1.3, 1, 0], [0, 0, 0, 1], test_world.root),
                    True,
                ),
                MoveTorsoActionDescription(TorsoState.HIGH),
                PlaceActionDescription(
                    test_world.get_body_by_name("milk.stl"),
                    PoseStamped.from_list(
                        [2.0, 1.6, 1.0], [0, 0, 0, 1], test_world.root
                    ),
                    Arms.LEFT,
                ),
            )

            sp.perform()
        dao = to_dao(sp)
        self.session.add(dao)
        self.session.commit()

    def test_exec_creation(self):
        plan = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root),
                True,
            ),
        )

        with simulated_robot:
            plan.perform()
        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        exec_data = self.session.scalars(select(ExecutionDataDAO)).all()
        self.assertIsNotNone(exec_data)

    def test_pose(self):
        plan = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root),
                True,
            ),
        )

        with simulated_robot:
            plan.perform()

        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        exec_data = self.session.scalars(select(ExecutionDataDAO)).all()[0]
        self.assertIsNotNone(exec_data)
        self.assertListEqual(
            [1.5, 2.5, 0],
            [
                exec_data.execution_start_pose.pose.position.x,
                exec_data.execution_start_pose.pose.position.y,
                exec_data.execution_start_pose.pose.position.z,
            ],
        )
        self.assertListEqual(
            [0.6, 0.4, 0],
            [
                exec_data.execution_end_pose.pose.position.x,
                exec_data.execution_end_pose.pose.position.y,
                exec_data.execution_end_pose.pose.position.z,
            ],
        )

    def test_manipulated_body_pose(self):
        test_world = deepcopy(self.world)

        self.plan(test_world)

        # pick_up = self.session.scalars(select(PickUpActionDAO)).all()[0]
        pick_up_node = self.session.scalars(
            select(ResolvedActionNodeMappingDAO).where(
                ResolvedActionNodeMappingDAO.designator_type == PickUpAction
            )
        ).all()[0]
        place_node = self.session.scalars(
            select(ResolvedActionNodeMappingDAO).where(
                ResolvedActionNodeMappingDAO.designator_type == PlaceAction
            )
        ).all()[0]
        # place = self.session.scalars(select(PlaceActionDAO)).all()[0]
        self.assertIsNotNone(pick_up_node.execution_data.manipulated_body_pose_start)
        self.assertIsNotNone(pick_up_node.execution_data.manipulated_body_pose_end)
        start_pose_pick = PoseStampedDAO.from_dao(
            pick_up_node.execution_data.manipulated_body_pose_start
        )
        end_pose_pick = PoseStampedDAO.from_dao(
            pick_up_node.execution_data.manipulated_body_pose_end
        )
        start_pose_place = PoseStampedDAO.from_dao(
            place_node.execution_data.manipulated_body_pose_start
        )
        end_pose_place = PoseStampedDAO.from_dao(
            place_node.execution_data.manipulated_body_pose_end
        )

        self.assertListEqual([2.37, 2, 1.05], start_pose_pick.position.to_list())
        # Check that the end_pose of pick_up and start pose of place are not equal because of navigate in between
        for pick, place in zip(
            end_pose_pick.position.to_list(), start_pose_place.position.to_list()
        ):
            self.assertNotEqual(pick, place)
        np.testing.assert_almost_equal([2.0, 1.6, 1], end_pose_place.position.to_list())

    def test_manipulated_body(self):
        test_world = deepcopy(self.world)

        self.plan(test_world)

        pick_up_node = self.session.scalars(
            select(ResolvedActionNodeMappingDAO).where(
                ResolvedActionNodeMappingDAO.designator_type == PickUpAction
            )
        ).all()[0]
        self.assertIsNotNone(pick_up_node.execution_data.manipulated_body)
        milk = BodyDAO.from_dao(pick_up_node.execution_data.manipulated_body)
        self.assertEqual(milk.name.name, "milk.stl")

    def test_state(self):
        plan = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root),
                True,
            ),
        )
        with simulated_robot:
            plan.perform()
        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()
        navigate_node = self.session.scalars(
            select(ResolvedActionNodeMappingDAO).where(
                ResolvedActionNodeMappingDAO.designator_type == NavigateAction
            )
        ).all()[0]
        self.assertIsNotNone(navigate_node.execution_data.execution_start_world_state)


class RelationalAlgebraTestCase(ORMaticBaseTestCaseMixin):
    def test_filtering(self):
        with simulated_robot:
            sp = SequentialPlan(
                self.context,
                NavigateActionDescription(
                    PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1], self.world.root),
                    True,
                ),
                ParkArmsActionDescription(Arms.BOTH),
                PickUpActionDescription(
                    self.world.get_body_by_name("milk.stl"),
                    Arms.LEFT,
                    GraspDescription(
                        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                    ),
                ),
                NavigateActionDescription(
                    PoseStamped.from_list([1.3, 1, 0.0], [0, 0, 0, 1], self.world.root),
                    True,
                ),
                PlaceActionDescription(
                    self.world.get_body_by_name("milk.stl"),
                    PoseStamped.from_list(
                        [2.0, 1.6, 0.9], [0, 0, 0, 1], self.world.root
                    ),
                    Arms.LEFT,
                ),
            )
            sp.perform()
        dao = to_dao(sp)
        self.session.add(dao)
        self.session.commit()

        filtered_navigate_results = self.session.scalars(
            select(NavigateActionDAO).where(NavigateActionDAO.database_id == 1)
        ).all()
        self.assertEqual(1, len(filtered_navigate_results))
