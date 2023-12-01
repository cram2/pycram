import threading
import unittest

from anytree import RenderTree

from pycram.designators.action_designator import *
from pycram.enums import ObjectType, State
from pycram.plan_failures import PlanFailure
from pycram.pose import Pose
from pycram.language import Sequential, Language, Parallel, TryAll, TryInOrder
from pycram.process_module import simulated_robot
import test_bullet_world

from pycram.language import Code


class LanguageTestCase(test_bullet_world.BulletWorldTest):

    def test_inheritance(self):
        act = NavigateAction([Pose()])
        self.assertTrue(issubclass(act.__class__, Language))

    def test_simplify_tree(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertEqual(len(plan.children), 3)
        self.assertEqual(plan.height, 1)

    def test_sequential_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertTrue(isinstance(plan, Sequential))
        self.assertEqual(len(plan.children), 3)

    def test_parallel_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act | act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 3)

    def test_try_in_order_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act - act2 - act3
        self.assertTrue(isinstance(plan, TryInOrder))
        self.assertEqual(len(plan.children), 3)

    def test_try_all_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act ^ act2 ^ act3
        self.assertTrue(isinstance(plan, TryAll))
        self.assertEqual(len(plan.children), 3)

    def test_combination_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        act3 = DetectAction([ObjectType.JEROEN_CUP])

        plan = act + act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 2)
        self.assertEqual(plan.height, 2)

    def test_code_construction(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])
        code = Code(lambda: True)

        plan = act + act2 + code
        self.assertEqual(len(plan.children), 3)
        self.assertEqual(plan.height, 1)

    def test_pickup_par_construction(self):
        act = NavigateAction([Pose()])
        act2 = PickUpAction(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(TypeError, lambda: act | act2)

    def test_pickup_try_all_construction(self):
        act = NavigateAction([Pose()])
        act2 = PickUpAction(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(TypeError, lambda: act ^ act2)

    def test_perform_desig(self):
        act = NavigateAction([Pose([1, 1, 0])])
        act2 = MoveTorsoAction([0.3])
        act3 = ParkArmsAction([Arms.BOTH])

        plan = act + act2 + act3
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.robot.get_pose(), Pose([1, 1, 0]))
        self.assertEqual(self.robot.get_joint_state("torso_lift_joint"), 0.3)
        for joint, pose in robot_description.get_static_joint_chain("right", "park").items():
            self.assertEqual(self.world.robot.get_joint_state(joint), pose)
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            self.assertEqual(self.world.robot.get_joint_state(joint), pose)

    def test_perform_code(self):
        def test_set(param):
            self.assertTrue(param)

        act = NavigateAction([Pose([1, 1, 0])])
        act2 = Code(test_set, {"param": True})
        act3 = Code(test_set, {"param": True})

        plan = act + act2 + act3
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.robot.get_pose(), Pose([1, 1, 0]))

    def test_perform_parallel(self):

        def check_thread_id(main_id):
            self.assertNotEquals(main_id, threading.get_ident())
        act = Code(check_thread_id, {"main_id": threading.get_ident()})
        act2 = Code(check_thread_id, {"main_id": threading.get_ident()})
        act3 = Code(check_thread_id, {"main_id": threading.get_ident()})

        plan = act | act2 | act3
        with simulated_robot:
            plan.perform()

    def test_exception_sequential(self):
        def raise_except():
            raise PlanFailure
        act = NavigateAction([Pose()])
        code = Code(raise_except)

        plan = act + code
        with simulated_robot:
            state = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.FAILED)

    def test_exception_try_in_order(self):
        def raise_except():
            raise PlanFailure
        act = NavigateAction([Pose()])
        code = Code(raise_except)

        plan = act - code
        with simulated_robot:
            state = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.SUCCEEDED)

    def test_exception_parallel(self):
        def raise_except():
            raise PlanFailure
        act = NavigateAction([Pose()])
        code = Code(raise_except)

        plan = act | code
        with simulated_robot:
            state = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.FAILED)

    def test_exception_try_all(self):
        def raise_except():
            raise PlanFailure
        act = NavigateAction([Pose()])
        code = Code(raise_except)

        plan = act ^ code
        with simulated_robot:
            state = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.SUCCEEDED)


if __name__ == '__main__':
    unittest.main()
