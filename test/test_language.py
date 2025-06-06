import threading
import time
import unittest
from pycram.designators.action_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import ObjectType, DetectionTechnique, TaskStatus, MonitorBehavior
from pycram.failure_handling import RetryMonitor
from pycram.fluent import Fluent
from pycram.failures import PlanFailure, NotALanguageExpression
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan, ParallelPlan, TryInOrderPlan, TryAllPLan, MonitorPlan, MonitorNode, \
    SequentialNode, RepeatPlan, CodePlan, TryAllNode
from pycram.process_module import simulated_robot
from pycram.testing import BulletWorldTestCase
from pycram.robot_description import RobotDescription
from pycrap.ontologies import Cup



class LanguageTestCase(BulletWorldTestCase):

    def test_simplify_tree(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = SequentialPlan(act, SequentialPlan(act2, act3))
        self.assertEqual(len(plan.root.children), 3)
        self.assertEqual(plan.root.children[0].children, [])

    def test_sequential_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = SequentialPlan(act, act2, act3)
        self.assertTrue(isinstance(plan, SequentialPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_parallel_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = ParallelPlan(act, act2, act3)
        self.assertTrue(isinstance(plan, ParallelPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_try_in_order_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = TryInOrderPlan(act, act2, act3)
        self.assertTrue(isinstance(plan, TryInOrderPlan))
        self.assertEqual(len(plan.root.children), 3)

    def test_try_all_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = TryAllPLan(act, act2, act3)
        self.assertTrue(TryAllNode, type(plan.root))
        self.assertTrue(isinstance(plan, TryAllPLan))
        self.assertEqual(len(plan.root.children), 3)

    def test_combination_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)
        act3 = DetectActionDescription(DetectionTechnique.TYPES)

        plan = ParallelPlan( SequentialPlan(act, act2), act3)
        self.assertTrue(isinstance(plan, ParallelPlan))
        self.assertEqual(len(plan.root.children), 2)
        self.assertTrue(isinstance(plan.root.children[0], SequentialNode))

    def test_pickup_par_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: ParallelPlan(act, act2))

    def test_pickup_try_all_construction(self):
        act = NavigateActionDescription(PoseStamped())
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: TryAllPLan(act, act2))

    def test_monitor_construction(self):
        act = ParkArmsActionDescription(Arms.BOTH)
        act2 = MoveTorsoActionDescription(TorsoState.HIGH)

        def monitor_func():
            time.sleep(1)
            return True

        plan = MonitorPlan(monitor_func, SequentialPlan(act, act2))
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

        subplan = MonitorPlan(monitor_func, SequentialPlan(act, act2))
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
        fail = CodePlan(raise_failure)
        counter = CodePlan(monitor_func)

        subplan = SequentialPlan(counter, fail)
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

        code = CodePlan(monitor_func)
        subplan = SequentialPlan(code)
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

        plan = RepeatPlan(5, SequentialPlan(act, act2))
        self.assertEqual(len(plan.root.children), 1)
        self.assertTrue(isinstance(plan.root.children[0], SequentialNode))

    def test_repeat_construction_error(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        park = ParkArmsActionDescription([Arms.BOTH])

        self.assertRaises(AttributeError, lambda: RepeatPlan(park, SequentialPlan(act, act2)))

    def test_perform_desig(self):
        act = NavigateActionDescription([PoseStamped.from_list([0.3, 0.3, 0])])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = ParkArmsActionDescription([Arms.BOTH])

        plan = SequentialPlan(act, act2, act3)
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0.3, 0]))
        self.assertEqual(self.robot.get_joint_position("torso_lift_joint"), 0.3)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right", StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left", StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)


    def test_perform_parallel(self):

        def check_thread_id(main_id):
            self.assertNotEqual(main_id, threading.get_ident())
        act = CodePlan(check_thread_id, {"main_id": threading.get_ident()})
        act2 = CodePlan(check_thread_id, {"main_id": threading.get_ident()})
        act3 = CodePlan(check_thread_id, {"main_id": threading.get_ident()})

        plan = ParallelPlan(act, act2, act3)
        with simulated_robot:
            plan.perform()

    def test_perform_repeat(self):
        test_var = Fluent(0)

        def inc(var):
            var.set_value(var.get_value() + 1)
        plan = RepeatPlan(10, CodePlan(lambda: inc(test_var)))
        with simulated_robot:
            plan.perform()
        self.assertEqual(10, test_var.get_value())

    def test_exception_sequential(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(raise_except)

        plan = SequentialPlan(act, code)
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
        code = CodePlan(raise_except)

        plan = TryInOrderPlan(act, code)
        with simulated_robot:
            _ = plan.perform()
        self.assertEqual(2, len(plan.root.children))
        self.assertEqual(TaskStatus.SUCCEEDED, plan.root.status)

    def test_exception_parallel(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(raise_except)

        plan = ParallelPlan(act, code)
        with simulated_robot:
            _ = plan.perform()
        self.assertEqual(PlanFailure, type(plan.root.reason))
        self.assertEqual(plan.root.status, TaskStatus.FAILED)

    def test_exception_try_all(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = CodePlan(raise_except)

        plan = TryAllPLan(act, code)
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

        plan = MonitorPlan(monitor_func, SequentialPlan(act, act2), behavior=MonitorBehavior.RESUME)
        with simulated_robot:
            plan.perform()
        self.assertEqual(len(plan.root.children), 1)
        self.assertTrue(isinstance(plan.root, MonitorNode))
        self.assertEqual(plan.root.status, TaskStatus.SUCCEEDED)
        # self.assertTrue(1 < (plan.root.end_time - plan.root.start_time).seconds <= 2)


if __name__ == '__main__':
    unittest.main()
