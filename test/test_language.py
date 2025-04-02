import threading
import time
import unittest
from pycram.designators.action_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import ObjectType, State
from pycram.failure_handling import RetryMonitor
from pycram.fluent import Fluent
from pycram.failures import PlanFailure, NotALanguageExpression
from pycram.datastructures.pose import PoseStamped
from pycram.language import Sequential, Language, Parallel, TryAll, TryInOrder, Monitor, Code
from pycram.process_module import simulated_robot
from pycram.testing import BulletWorldTestCase
from pycram.robot_description import RobotDescription


class LanguageTestCase(BulletWorldTestCase):

    def test_inheritance(self):
        act = NavigateActionDescription([PoseStamped()])
        self.assertTrue(issubclass(act.__class__, Language))

    def test_simplify_tree(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertEqual(len(plan.children), 3)
        self.assertEqual(plan.height, 1)

    def test_sequential_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act + act2 + act3
        self.assertTrue(isinstance(plan, Sequential))
        self.assertEqual(len(plan.children), 3)

    def test_parallel_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act | act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 3)

    def test_try_in_order_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act - act2 - act3
        self.assertTrue(isinstance(plan, TryInOrder))
        self.assertEqual(len(plan.children), 3)

    def test_try_all_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act ^ act2 ^ act3
        self.assertTrue(isinstance(plan, TryAll))
        self.assertEqual(len(plan.children), 3)

    def test_combination_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = DetectActionDescription([ObjectType.JEROEN_CUP])

        plan = act + act2 | act3
        self.assertTrue(isinstance(plan, Parallel))
        self.assertEqual(len(plan.children), 2)
        self.assertEqual(plan.height, 2)

    def test_code_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        code = Code(lambda: True)

        plan = act + act2 + code
        self.assertEqual(len(plan.children), 3)
        self.assertEqual(plan.height, 1)

    def test_pickup_par_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: act | act2)

    def test_pickup_try_all_construction(self):
        act = NavigateActionDescription([PoseStamped()])
        act2 = PickUpActionDescription(BelieveObject(names=["milk"]), ["left"], ["front"])

        self.assertRaises(AttributeError, lambda: act ^ act2)

    def test_monitor_construction(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])

        def monitor_func():
            time.sleep(1)
            return True

        plan = act + act2 >> Monitor(monitor_func)
        self.assertEqual(len(plan.children), 1)
        self.assertEqual(plan.height, 2)

    def test_monitor_construction_error(self):

        def monitor_func():
            time.sleep(1)
            return True

        self.assertRaises(AttributeError, lambda: Monitor(monitor_func) >> Monitor(monitor_func))

    def test_retry_monitor_construction(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])

        def monitor_func():
            time.sleep(1)
            return True

        def recovery1():
            return

        recover1 = Code(lambda: recovery1())
        recovery = {NotALanguageExpression: recover1}

        subplan = act + act2 >> Monitor(monitor_func)
        plan = RetryMonitor(subplan, max_tries=6, recovery=recovery)
        self.assertEqual(len(plan.recovery), 1)
        self.assertIsInstance(plan.designator_description, Monitor)

    def test_retry_monitor_tries(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        tries_counter = 0

        def monitor_func():
            nonlocal tries_counter
            tries_counter += 1
            return True

        subplan = act + act2 >> Monitor(monitor_func)
        plan = RetryMonitor(subplan, max_tries=6)
        try:
            plan.perform()
        except PlanFailure as e:
            pass
        self.assertEqual(tries_counter, 6)

    def test_retry_monitor_recovery(self):
        recovery1_counter = 0
        recovery2_counter = 0

        def monitor_func():
            if not hasattr(monitor_func, 'tries_counter'):
                monitor_func.tries_counter = 0
            if monitor_func.tries_counter % 2:
                monitor_func.tries_counter += 1
                return NotALanguageExpression
            monitor_func.tries_counter += 1
            return PlanFailure

        def recovery1():
            nonlocal recovery1_counter
            recovery1_counter += 1

        def recovery2():
            nonlocal recovery2_counter
            recovery2_counter += 1

        recover1 = Code(lambda: recovery1())
        recover2 = Code(lambda: recovery2())
        recovery = {NotALanguageExpression: recover1,
                    PlanFailure: recover2}

        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        subplan = act + act2 >> Monitor(monitor_func)
        plan = RetryMonitor(subplan, max_tries=6, recovery=recovery)
        try:
            plan.perform()
        except PlanFailure as e:
            pass
        self.assertEqual(recovery1_counter, 2)
        self.assertEqual(recovery2_counter, 3)

    def test_repeat_construction(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])

        plan = (act + act2) * 5
        self.assertEqual(len(plan.children), 1)
        self.assertEqual(plan.height, 2)

    def test_repeat_construction_reverse(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])

        plan = 5 * (act + act2)
        self.assertEqual(len(plan.children), 1)
        self.assertEqual(plan.height, 2)

    def test_repeat_construction_error(self):
        act = ParkArmsActionDescription([Arms.BOTH])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        park = ParkArmsActionDescription([Arms.BOTH])

        self.assertRaises(AttributeError, lambda: (act + act2) * park)

    def test_perform_desig(self):
        act = NavigateActionDescription([PoseStamped.from_list([0.3, 0.3, 0])])
        act2 = MoveTorsoActionDescription([TorsoState.HIGH])
        act3 = ParkArmsActionDescription([Arms.BOTH])

        plan = act + act2 + act3
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0.3, 0]))
        self.assertEqual(self.robot.get_joint_position("torso_lift_joint"), 0.3)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right", StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left", StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)

    def test_perform_code(self):
        def test_set(param):
            self.assertTrue(param)

        act = NavigateActionDescription([PoseStamped.from_list([0.3, 0.3, 0])])
        act2 = Code(test_set, {"param": True})
        act3 = Code(test_set, {"param": True})

        plan = act + act2 + act3
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0.3, 0]))

    def test_perform_parallel(self):

        def check_thread_id(main_id):
            self.assertNotEqual(main_id, threading.get_ident())
        act = Code(check_thread_id, {"main_id": threading.get_ident()})
        act2 = Code(check_thread_id, {"main_id": threading.get_ident()})
        act3 = Code(check_thread_id, {"main_id": threading.get_ident()})

        plan = act | act2 | act3
        with simulated_robot:
            plan.perform()

    def test_perform_repeat(self):
        test_var = Fluent(0)

        def inc(var):
            var.set_value(var.get_value() + 1)
        plan = Code(lambda: inc(test_var)) * 10
        with simulated_robot:
            plan.perform()
        self.assertEqual(10, test_var.get_value())

    def test_exception_sequential(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = Code(raise_except)

        plan = act + code
        with simulated_robot:
            state, _ = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.FAILED)

    def test_exception_try_in_order(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = Code(raise_except)

        plan = act - code
        with simulated_robot:
            state, _ = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.SUCCEEDED)

    def test_exception_parallel(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = Code(raise_except)

        plan = act | code
        with simulated_robot:
            state, _ = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.FAILED)

    def test_exception_try_all(self):
        def raise_except():
            raise PlanFailure
        act = NavigateActionDescription([PoseStamped()])
        code = Code(raise_except)

        plan = act ^ code
        with simulated_robot:
            state, _ = plan.perform()
        self.assertIsInstance(plan.exceptions[plan], list)
        self.assertIsInstance(plan.exceptions[plan][0], PlanFailure)
        self.assertEqual(len(plan.exceptions.keys()), 1)
        self.assertEqual(state, State.SUCCEEDED)


if __name__ == '__main__':
    unittest.main()
