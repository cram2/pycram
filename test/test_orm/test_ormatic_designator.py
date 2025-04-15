import logging
import os
import sys
import unittest

import anytree
import sqlalchemy
from sqlalchemy import create_engine, select, text
from sqlalchemy.orm import registry, Session, clear_mappers
import sqlalchemy.sql.elements

import pycram.tasktree
from pycram.datastructures.dataclasses import FrozenObject
from pycram.datastructures.enums import TorsoState, Arms, Grasp, DetectionTechnique
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import Vector3, Quaternion, Pose, PoseStamped
from pycram.designator import ObjectDesignatorDescription, ActionDescription
from pycram.designators.action_designator import PickUpActionDescription, MoveTorsoActionDescription, ParkArmsAction, \
    MoveTorsoAction, NavigateAction, GraspingAction, PickUpAction, DetectActionDescription, DetectAction, \
    PlaceActionDescription, TransportAction, PlaceAction
from pycram.failures import ObjectNotGraspedError, PerceptionObjectNotFound
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescription
from pycram.tasktree import with_tree, task_tree
from pycram.testing import BulletWorldTestCase

from pycram.orm.logging_hooks import insert
from pycram.orm.ormatic_interface import mapper_registry
from pycram.orm.model import TaskTreeNode as TaskTreeNodeORM, PickUpAction as PickUpActionORM

class ORMaticBaseTestCaseMixin(BulletWorldTestCase):
    engine: sqlalchemy.engine
    session: Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = create_engine('sqlite:///:memory:')

    def setUp(self):
        super().setUp()
        self.mapper_registry = mapper_registry
        self.session = Session(self.engine)
        self.mapper_registry.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        self.mapper_registry.metadata.drop_all(self.session.bind)
        # clear_mappers()
        self.session.close()
        pycram.tasktree.task_tree.reset_tree()


class SchemaTestCases(ORMaticBaseTestCaseMixin):
    def test_schema_creation(self):
        tables = list(self.mapper_registry.metadata.tables.keys())
        self.assertTrue("Vector3" in tables)
        self.assertTrue("Quaternion" in tables)
        self.assertTrue("Pose" in tables)
        self.assertTrue("Header" in tables)
        self.assertTrue("GraspDescription" in tables)
        self.assertTrue("FrozenObject" in tables)
        self.assertTrue("PoseStamped" in tables)
        self.assertTrue("Transform" in tables)
        self.assertTrue("TransformStamped" in tables)
        self.assertTrue("CloseAction" in tables)
        self.assertTrue("DetectAction" in tables)
        self.assertTrue("FaceAtAction" in tables)
        self.assertTrue("GraspingAction" in tables)
        self.assertTrue("GripAction" in tables)
        self.assertTrue("LookAtAction" in tables)
        self.assertTrue("MoveTorsoAction" in tables)
        self.assertTrue("NavigateAction" in tables)
        self.assertTrue("OpenAction" in tables)
        self.assertTrue("ParkArmsAction" in tables)
        self.assertTrue("PickUpAction" in tables)
        self.assertTrue("PlaceAction" in tables)
        self.assertTrue("ReleaseAction" in tables)
        self.assertTrue("SetGripperAction" in tables)
        self.assertTrue("TaskTreeNode" in tables)
        self.assertTrue("TransportAction" in tables)

class PoseTestCases(ORMaticBaseTestCaseMixin):
    @with_tree
    def plan(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = PlaceActionDescription(object_description, [PoseStamped
                                             .from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        torso_joint = RobotDescription.current_robot_description.torso_joint
        self.assertEqual(description.resolve().object_designator.name, "milk")
        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoAction(TorsoState.HIGH).perform()
            grasp = GraspDescription(Grasp.FRONT, None, False)
            PickUpAction(object_description.resolve(), Arms.LEFT, grasp, 0.03).perform()
            description.resolve().perform()

    def test_pose(self):
        self.plan()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(Pose)).all()
        self.assertTrue(all([r.position is not None and r.orientation is not None for r in result]))

    def test_action_to_pose(self):
        self.plan()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(ActionDescription)).all()
        self.assertTrue(all([r.robot_position.pose is not None and r.robot_position.pose_id == r.robot_position.pose.id for r in result]))

    def test_place_action(self):
        self.plan()
        insert(task_tree.root, self.session)
        x = self.session.scalars(select(TaskTreeNodeORM)).all()
        result = self.session.scalars(select(PlaceAction)).all()
        print(result)
        self.assertTrue(len(result) == 1)

class ORMActionDesignatorTestCase(ORMaticBaseTestCaseMixin):
    def test_code_designator_type(self):
        action = NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True)
        with simulated_robot:
            action.perform()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(TaskTreeNodeORM).where(TaskTreeNodeORM.action_id.isnot(None))).all()
        self.assertEqual(type(result[0].action), NavigateAction)
        # self.assertEqual(result[1].action.dtype, MoveMotion.__name__)

    def test_parkArmsAction(self):
        action = ParkArmsAction(pycram.datastructures.enums.Arms.BOTH)
        with simulated_robot:
            action.perform()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(ParkArmsAction)).all()
        self.assertEqual(len(result), 1)
        self.assertEqual(type(result[0]), ParkArmsAction)

    @unittest.skip
    def test_transportAction(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        action = TransportAction(object_description.resolve(),
                                 PoseStamped.from_list([1.3, 0.9, 0.9], [0, 0, 0, 1]), Arms.LEFT, 0.03)
        with simulated_robot:
            action.perform()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(TransportAction)).all()
        self.assertEqual(type(result), TransportAction)
        self.assertTrue(result[0].target_location is not None)
        milk_object = self.session.scalars(select(FrozenObject)).first()
        self.assertEqual(milk_object.pose, result[0].object_at_execution.pose)

    def test_pickUpAction(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        previous_position = object_description.resolve().pose
        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            ParkArmsAction(Arms.BOTH).perform()
            grasp = GraspDescription(Grasp.FRONT, None, False)
            PickUpAction(object_description.resolve(), Arms.LEFT, grasp, 0.03).perform()
            NavigateAction(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True).perform()
            PlaceAction(object_description.resolve(), PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                        Arms.LEFT).perform()
        insert(task_tree.root, self.session)
        result = self.session.scalars(select(Vector3)
                                      .join(PickUpAction.object_at_execution)
                                      .join(FrozenObject.pose)
                                      .join(PoseStamped.pose)
                                      .join(Pose.position)).first()
        self.assertEqual(result.x, previous_position.position.x)
        self.assertEqual(result.y, previous_position.position.y)
        self.assertEqual(result.z, previous_position.position.z)

    # # TODO: dicuss on how to change this
    # @unittest.skip
    # def test_lookAt_and_detectAction(self):
    #     object_description = ObjectDesignatorDescription(types=[Milk])
    #     action = DetectActionDescription(technique=DetectionTechnique.TYPES,
    #                                      state=DetectionState.START,
    #                                      object_designator_description=object_description,
    #                                      region=None).resolve()
    #     with simulated_robot:
    #         ParkArmsAction(pycram.datastructures.enums.Arms.BOTH).perform()
    #         NavigateAction(PoseStamped.from_list([0, 1, 0], [0, 0, 0, 1]), True).perform()
    #         LookAtAction(object_description.resolve().pose).perform()
    #         action.perform()
    #     pycram.orm.base.ProcessMetaData().description = "detectAction_test"
    #     pycram.tasktree.task_tree.root.insert(self.session)
    #     result = self.session.scalars(select(pycram.orm.action_designator.DetectAction)).all()
    #     self.assertEqual(result[0].object.name, "milk")
    #
    # def test_setGripperAction(self):
    #     action = SetGripperAction(Arms.LEFT, GripperState.OPEN)
    #     with simulated_robot:
    #         action.perform()
    #     pycram.orm.base.ProcessMetaData().description = "setGripperAction_test"
    #     pycram.tasktree.task_tree.root.insert(self.session)
    #     result = self.session.scalars(select(pycram.orm.action_designator.SetGripperAction)).all()
    #     self.assertEqual(result[0].gripper, Arms.LEFT)
    #     self.assertEqual(result[0].motion, GripperState.OPEN)
    #
    # def test_open_and_closeAction(self):
    #     apartment = Object("apartment", Apartment, "apartment.urdf")
    #     apartment_desig = BelieveObject(names=["apartment"]).resolve()
    #     handle_desig = object_designator.ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig).resolve()
    #
    #     self.kitchen.set_pose(PoseStamped.from_list([20, 20, 0], [0, 0, 0, 1]))
    #
    #     with simulated_robot:
    #         ParkArmsAction(pycram.datastructures.enums.Arms.BOTH).perform()
    #         NavigateAction(PoseStamped.from_list([1.81, 1.73, 0.0],
    #                                              [0.0, 0.0, 0.594, 0.804]), True).perform()
    #         OpenAction(handle_desig, arm=Arms.LEFT, grasping_prepose_distance=0.03).perform()
    #         CloseAction(handle_desig, arm=Arms.LEFT, grasping_prepose_distance=0.03).perform()
    #
    #     pycram.orm.base.ProcessMetaData().description = "open_and_closeAction_test"
    #     pycram.tasktree.task_tree.root.insert(self.session)
    #     open_result = self.session.scalars(select(pycram.orm.action_designator.OpenAction)).all()
    #     close_result = self.session.scalars(select(pycram.orm.action_designator.CloseAction)).all()
    #     self.assertTrue(open_result is not None)
    #     self.assertEqual(open_result[0].object.name, "handle_cab10_t")
    #     self.assertTrue(close_result is not None)
    #     self.assertEqual(close_result[0].object.name, "handle_cab10_t")
    #     apartment.remove()
    # @with_tree
    # def plan(self):
    #     # action = ParkArmsAction(Arms.BOTH)
    #     with simulated_robot:
    #         MoveTorsoAction(TorsoState.HIGH).perform()
    #         ParkArmsAction(Arms.BOTH).perform()

    # @with_tree
    # def plan(self):
    #     object_description = ObjectDesignatorDescription(names=["milk"])
    #     description = PlaceActionDescription(object_description, [PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
    #     torso_joint = RobotDescription.current_robot_description.torso_joint
    #     # self.assertEqual(description.resolve().object_designator.name, "milk")
    #     with simulated_robot:
    #         NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
    #         MoveTorsoAction(TorsoState.HIGH).perform()
    #         grasp = GraspDescription(Grasp.FRONT, None, False)
    #         PickUpAction(object_description.resolve(), Arms.LEFT, grasp, 0.03).perform()
    #         description.resolve().perform()

    # @with_tree
    # def plan(self):
    #     object_description = ObjectDesignatorDescription(names=["milk"])
    #     grasp_description = GraspDescription(Grasp.FRONT, None, False)
    #     description = PickUpActionDescription(object_description, [Arms.LEFT], [grasp_description])
    #     self.assertEqual(description.resolve().object_designator.name, "milk")
    #     with simulated_robot:
    #         NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
    #         MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
    #         description.resolve().perform()




    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        with simulated_robot:
            self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
            self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
            # object_description = ObjectDesignatorDescription(types=[Milk])
            # pr2 = Object("pr2", Robot, "pr2.urdf", pose=PoseStamped.from_list([1, 2, 0]))
            description = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                                  object_designator=self.milk)
            description.resolve().perform()

        print(anytree.RenderTree(pycram.tasktree.TaskTree().root))
        insert(pycram.tasktree.task_tree.root, self.session)
        node_results = self.session.scalars(select(TaskTreeNodeORM)).all()
        # frozen_results = self.session.scalars(select(FrozenObject)).all()
        print(node_results)
        self.assertEqual(len(node_results), len(pycram.tasktree.task_tree.root))

        action_results = self.session.scalars(select(ActionDescription)).all()
        print(action_results)
        self.assertEqual(1, len(action_results))

        detect_actions = self.session.scalars(select(DetectAction)).all()
        print(detect_actions)
        self.assertEqual(1, len(detect_actions))

    def test_general_orm_designator_creation(self):
        with simulated_robot:
            MoveTorsoAction(TorsoState.HIGH).perform()
            ParkArmsAction(Arms.BOTH).perform()

        insert(pycram.tasktree.TaskTree().root, self.session)

        y = self.session.scalars(select(Vector3)).all()
        z = self.session.scalars(select(Quaternion)).all()
        a = self.session.scalars(select(PoseStamped)).all()
        b = self.session.scalars(select(PoseStamped.pose_id)).all()
        # print(x)
        print(y)
        print(z)
        result = self.session.scalars(select(ParkArmsAction).join(PoseStamped)).all()
        result2 = self.session.scalars(select(MoveTorsoAction)).all()

        print(result)
        # print(result2)
        self.assertEqual(len(result), 1)
        self.assertEqual(len(result2), 1)

    def test_pose_creation(self):
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 3.0
        pose.orientation.x = 4.0
        pose.orientation.y = 5.0
        pose.orientation.z = 6.0
        pose.orientation.w = 7.0

        print(pose)
        self.session.add(pose.position)
        self.session.add(pose.orientation)
        self.session.add(pose)
        self.session.commit()

        vectors = self.session.scalars(select(Vector3)).all()
        print(vectors)
        orientations = self.session.scalars(select(Quaternion)).all()
        print(orientations)

        with self.session.bind.connect() as conn:
            raw_pose = conn.execute(text("SELECT * FROM Pose")).fetchall()
        print(raw_pose)

        poses = self.session.scalars(select(Pose)).all()
        print(poses)
        self.assertEqual(len(poses), 1)
    #
    # def test_self_mapping_orm_designators(self):
    #     pass
    #
    # def test_explicitly_mapped_orm_designators(self):
    #     pass
    #
    # def test_frozen_object_mapping(self):
    #     pass
    #
