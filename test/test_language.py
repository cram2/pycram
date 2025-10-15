import threading
import time
import unittest

from pycram.datastructures.enums import TaskStatus, MonitorBehavior
from pycram.failure_handling import RetryMonitor
from pycram.failures import PlanFailure, NotALanguageExpression
from pycram.fluent import Fluent
from pycram.language import ParallelPlan, TryAllPLan, MonitorPlan, MonitorNode, \
    SequentialNode, RepeatPlan, CodePlan, TryAllNode
from pycram.process_module import simulated_robot
from pycram.robot_plans import *
from pycram.testing import BulletWorldTestCase


class LanguageTestCase(BulletWorldTestCase):

    def test_simplify_tree(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = SequentialPlan(self.context, self.robot_view, act, SequentialPlan(self.context, self.robot_view, act2, act3))
        self.assertEqual(len(plan.root.children), 3)
        self.assertEqual(plan.root.children[0].children, [])

    def test_sequential_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = SequentialPlan(self.context, self.robot_view, act, act2, act3)
        self.assertTrue(isinstance(plan, SequentialPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_parallel_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = ParallelPlan(self.context, self.robot_view, act, act2, act3)
        self.assertTrue(isinstance(plan, ParallelPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_try_in_order_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = TryInOrderPlan(self.context, self.robot_view, act, act2, act3)
        self.assertTrue(isinstance(plan, TryInOrderPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_try_all_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = TryAllPLan(self.context, self.robot_view, act, act2, act3)
        self.assertTrue(TryAllNode, type(plan.root))
        self.assertTrue(isinstance(plan, TryAllPLan))
        self.assertEqual(len(plan.root.children), 3)

    def test_combination_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = ParallelPlan(self.context, self.robot_view, SequentialPlan(self.context, self.robot_view, act, act2),
                            act3)
        self.assertTrue(isinstance(plan, ParallelPlan))
        self.assertEqual(len(plan.root.children), 2)
        self.assertTrue(isinstance(plan.root.children[0], SequentialNode))

    def test_pickup_par_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: ParallelPlan(self.context, act, act2))

    def test_pickup_try_all_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: TryAllPLan(self.context, self.robot_view, act, act2))

    def test_monitor_construction(self):
        act = ParkArmsActionDescription(Arms.BOTH)
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)

        def monitor_func():
            time.sleep(1)
            return True

        plan = MonitorPlan(monitor_func, self.context, self.robot_view, SequentialPlan(self.context, act, act2))
        self.assertEqual(len(plan.root.children), 1)
        self.assertTrue(isinstance(plan.root, MonitorNode))

    def test_retry_monitor_construction(self):
        act = ParkArmsActionDescription(Arms.BOTH)
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)

        def monitor_func():
            time.sleep(1)
            return True

        def recovery1():
            return

        recovery = {NotALanguageExpression: recovery1}

        subplan = MonitorPlan(monitor_func, self.context, self.robot_view,
                              SequentialPlan(self.context, self.robot_view, act, act2))
        plan = RetryMonitor(subplan, max_tries=6, recovery=recovery)
        self.assertEqual(len(plan.recovery), 1)
        self.assertIsInstance(plan.plan, MonitorPlan)

    def test_retry_monitor_tries(self):
        def raise_failure():
            raise PlanFailure

        tries_counter = 0

        def monitor_func():
            nonlocal tries_counter
            tries_counter += 1
            return True

        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        fail = CodePlan(self.context, self.robot_view, raise_failure)
        counter = CodePlan(self.context, self.robot_view, monitor_func)

        subplan = SequentialPlan(self.context, self.robot_view, counter, fail)
        plan = RetryMonitor(subplan, max_tries=6)
        self.assertRaises(PlanFailure, plan.perform)

        self.assertEqual(tries_counter, 6)

    def test_retry_monitor_recovery(self):
        recovery1_counter = 0
        recovery2_counter = 0

        def monitor_func():
            if not hasattr(monitor_func, 'tries_counter'):
                monitor_func.tries_counter = 0
            if monitor_func.tries_counter % 2:
                monitor_func.tries_counter += 1
                raise NotALanguageExpression
            monitor_func.tries_counter += 1
            raise PlanFailure

        def recovery1():
            nonlocal recovery1_counter
            recovery1_counter += 1

        def recovery2():
            nonlocal recovery2_counter
            recovery2_counter += 1

        recovery = {NotALanguageExpression: recovery1,
                    PlanFailure: recovery2}

        code = CodePlan(self.context, self.robot_view, monitor_func)
        subplan = SequentialPlan(self.context, self.robot_view, code)
        plan = RetryMonitor(subplan, max_tries=6, recovery=recovery)
        try:
            plan.perform()
        except PlanFailure as e:
            pass
        self.assertEqual(2, recovery1_counter)
        self.assertEqual(3, recovery2_counter)

    def test_repeat_construction(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])

        plan = RepeatPlan(self.context, self.robot_view, 5, SequentialPlan(self.context, self.robot_view, act, act2))
        self.assertEqual(len(plan.root.children), 1)
        self.assertTrue(isinstance(plan.root.children[0], SequentialNode))

    def test_repeat_construction_error(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        park = ParkArmsActionDescription([Arms.BOTH])

        self.assertRaises(AttributeError,
                          lambda: RepeatPlan(self.context, self.robot_view, park,
                                             SequentialPlan(self.context, self.robot_view, act, act2)))

    def test_perform_desig(self):
        act = NavigateActionDescription([PoseStamped.from_list( [0.3, 0.3, 0], frame=self.world.root)])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = ParkArmsActionDescription([Arms.BOTH])

        plan = SequentialPlan(self.context, self.robot_view, act, act2, act3)
        with simulated_robot:
            plan.perform()
        self.assertEqual(PoseStamped.from_spatial_type(self.robot_view.root.global_pose), PoseStamped.from_list( [0.3, 0.3, 0], frame=self.world.root))
        self.assertEqual(self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position, 0.3)


    def test_perform_parallel(self):

        def check_thread_id(main_id):
            self.assertNotEqual(main_id, threading.get_ident())

        act = CodePlan(self.context, self.robot_view, check_thread_id, {"main_id": threading.get_ident()})
        act2 = CodePlan(self.context, self.robot_view, check_thread_id, {"main_id": threading.get_ident()})
        act3 = CodePlan(self.context, self.robot_view, check_thread_id, {"main_id": threading.get_ident()})

        plan = ParallelPlan(self.context, self.robot_view, act, act2, act3)
        with simulated_robot:
            plan.perform()

    def test_perform_repeat(self):
        test_var = Fluent(0)

        def inc(var):
            var.set_value(var.get_value() + 1)

        plan = RepeatPlan(self.context, self.robot_view, 10, CodePlan(self.context, self.robot_view, lambda: inc(test_var)))
        with simulated_robot:
            plan.perform()
        self.assertEqual(10, test_var.get_value())

    def test_exception_sequential(self):
        def raise_except():
            raise PlanFailure

        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(self.context, self.robot_view, raise_except)

        plan = SequentialPlan(self.context, self.robot_view, act, code)

        def perform_plan():
            with simulated_robot:
                _ = plan.perform()

        self.assertRaises(PlanFailure, perform_plan)
        self.assertEqual(2, len(plan.root.children))
        self.assertEqual(TaskStatus.FAILED, plan.root.status)

    def test_exception_try_in_order(self):
        def raise_except():
            raise PlanFailure

        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(self.context, self.robot_view, raise_except)

        plan = TryInOrderPlan(self.context, self.robot_view, act, code)
        with simulated_robot:
            _ = plan.perform()
        self.assertEqual(2, len(plan.root.children))
        self.assertEqual(TaskStatus.SUCCEEDED, plan.root.status)

    def test_exception_parallel(self):
        def raise_except():
            raise PlanFailure

        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(self.context, self.robot_view, raise_except)

        plan = ParallelPlan(self.context, self.robot_view, act, code)
        with simulated_robot:
            _ = plan.perform()
        self.assertEqual(PlanFailure, type(plan.root.reason))
        self.assertEqual(plan.root.status, TaskStatus.FAILED)

    def test_exception_try_all(self):
        def raise_except():
            raise PlanFailure

        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(self.context, self.robot_view, raise_except)

        plan = TryAllPLan(self.context, self.robot_view, act, code)
        with simulated_robot:
            _ = plan.perform()

        self.assertEqual(TryAllNode, type(plan.root))
        self.assertEqual(TaskStatus.SUCCEEDED, plan.root.status)

    def test_monitor_resume(self):
        act = ParkArmsActionDescription(Arms.BOTH)
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)

        def monitor_func():
            time.sleep(2)
            return True

        plan = MonitorPlan(monitor_func, self.context, self.robot_view,
                           SequentialPlan(self.context, self.robot_view, act, act2),
                           behavior=MonitorBehavior.RESUME)
        with simulated_robot:
            plan.perform()
        self.assertEqual(len(plan.root.children), 1)
        self.assertTrue(isinstance(plan.root, MonitorNode))
        self.assertEqual(plan.root.status, TaskStatus.SUCCEEDED)
        # self.assertTrue(1 < (plan.root.end_time - plan.root.start_time).seconds <= 2)


if __name__ == '__main__':
    unittest.main()
