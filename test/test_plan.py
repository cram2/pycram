import os
import time
import unittest
from datetime import datetime

from random_events.interval import closed
from random_events.product_algebra import SimpleEvent, Event
from semantic_digital_twin.adapters.urdf import URDFParser

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus
from pycram.robot_plans import *
from pycram.language import SequentialPlan, ParallelPlan, CodeNode
from pycram.parameterizer import Parameterizer
from pycram.plan import PlanNode, Plan
from pycram.process_module import simulated_robot
from pycram.testing import BulletWorldTestCase


class TestPlan(unittest.TestCase):

    def setUp(self):
        Plan.current_plan = None
        self.world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "resources", "robots", "pr2.urdf")).parse()
        self.context = Context(self.world, None, None)

    def test_plan_construction(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        self.assertEqual(node, plan.root)
        self.assertEqual(len(plan.edges), 0)
        self.assertEqual(len(plan.nodes), 1)
        self.assertEqual(plan, node.plan)
        self.assertIsNone(Plan.current_plan)

    def test_add_edge(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        node2 = PlanNode()
        plan.add_edge(node, node2)
        self.assertEqual(node, plan.root)
        self.assertIn(node, plan.nodes)
        self.assertEqual(len(plan.nodes), 2)
        self.assertEqual(len(plan.edges), 1)
        self.assertIn(node2, plan.nodes)
        self.assertEqual(plan, node2.plan)
        self.assertEqual(plan, node2.plan)

    def test_add_node(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        node2 = PlanNode()
        plan.add_node(node2)
        self.assertEqual(node, plan.root)
        self.assertIn(node, plan.nodes)
        self.assertIn(node2, plan.nodes)
        self.assertNotIn((node, node2), plan.edges)
        self.assertEqual(plan, node2.plan)

    def test_mount(self):
        plan1_node = PlanNode()
        plan1 = Plan(plan1_node, self.context)
        plan2_node = PlanNode()
        plan2 = Plan(plan2_node, self.context)

        plan1.mount(plan2)
        self.assertIn(plan2_node, plan1.nodes)
        self.assertEqual(plan1, plan2_node.plan)
        self.assertEqual(plan1, plan2_node.plan)
        self.assertEqual(len(plan1.edges), 1)
        self.assertEqual(len(plan1.nodes), 2)

    def test_mount_specific_node(self):
        plan = Plan(PlanNode(), self.context)
        mount_node = PlanNode()
        plan.add_edge(plan.root, mount_node)

        plan2 = Plan(PlanNode(), self.context)
        plan.mount(plan2, mount_node)

        self.assertIn(plan2.root, plan.nodes)
        self.assertEqual(plan, plan2.root.plan)
        self.assertIn((mount_node, plan2.root), plan.edges)
        self.assertEqual(len(plan.edges), 2)
        self.assertEqual(len(plan.nodes), 3)

    def test_context_creation(self):
        super_plan = Plan(PlanNode(), self.context)
        context = Context(self.world, 1, super_plan)
        plan = Plan(PlanNode(), context)
        self.assertEqual(context, plan.context)
        self.assertEqual(plan.world, self.world)
        self.assertEqual(plan.robot, 1)
        self.assertEqual(plan.super_plan, super_plan)


class TestPlanNode(unittest.TestCase):
    def setUp(self):
        self.world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "resources", "robots", "pr2.urdf")).parse()
        self.context = Context(self.world, None, None)

    def test_plan_node_creation(self):
        node = PlanNode()
        self.assertIsInstance(node, PlanNode)
        self.assertEqual(node.status, TaskStatus.CREATED)
        self.assertEqual(node.plan, None)
        self.assertLessEqual(node.start_time, datetime.datetime.now())

    def test_plan_node_parent(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        node2 = PlanNode()
        plan.add_edge(node, node2)

        self.assertIsNone(node.parent)
        self.assertEqual(node, node2.parent)

    def test_plan_all_parents(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        node2 = PlanNode()
        plan.add_edge(node, node2)
        node3 = PlanNode()
        plan.add_edge(node2, node3)

        self.assertEqual(node.all_parents, [])
        self.assertEqual(node2.all_parents, [node])
        self.assertEqual(node3.all_parents, [node2, node])

    def test_plan_node_children(self):
        node = PlanNode()
        plan = Plan(node, self.context)

        self.assertEqual([], node.children)

        node2 = PlanNode()
        plan.add_edge(node, node2)
        self.assertEqual([node2], node.children)

        node3 = PlanNode()
        plan.add_edge(node, node3)
        self.assertEqual([node2, node3], node.children)

    def test_plan_node_recursive_children(self):
        node = PlanNode()
        plan = Plan(node,self.context)

        self.assertEqual([], node.recursive_children)

        node2 = PlanNode()
        plan.add_edge(node, node2)
        self.assertEqual([node2], node.recursive_children)

        node3 = PlanNode()
        plan.add_edge(node2, node3)
        self.assertEqual([node2, node3], node.recursive_children)

    def test_plan_node_is_leaf(self):
        node = PlanNode()
        plan = Plan(node, self.context)
        node2 = PlanNode()
        plan.add_edge(node, node2)

        self.assertFalse(node.is_leaf)
        self.assertTrue(node2.is_leaf)

    def test_plan_node_subtree(self):
        node = PlanNode()
        node2 = PlanNode()
        node3 = PlanNode()
        plan = Plan(node, self.context)
        plan.add_edge(node, node2)
        plan.add_edge(node2, node3)

        sub_tree = node2.subtree
        self.assertEqual(node2, sub_tree.root)
        self.assertIn(node2, sub_tree.nodes)
        self.assertIn(node3, sub_tree.nodes)
        self.assertEqual(len(sub_tree.edges), 1)
        self.assertIn((node2, node3), sub_tree.edges)

class TestPlanInterrupt(BulletWorldTestCase):
        def test_interrupt_plan(self):

            def interrupt_plan():
                Plan.current_plan.root.interrupt()

            code_node = CodeNode(interrupt_plan)
            with simulated_robot:
                SequentialPlan(self.context, MoveTorsoActionDescription(TorsoState.HIGH), Plan(code_node, self.context), MoveTorsoActionDescription([TorsoState.LOW])).perform()

            self.assertEqual(0.3, self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position)

        @unittest.skip(
            "There is some weird error here that causes the interpreter to abort with exit code 134, something with thread handling. Needs more investigation")
        def test_pause_plan(self):
            def node_sleep():
                sleep(1)

            def pause_plan():
                Plan.current_plan.root.pause()
                self.assertEqual(0, self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position)
                Plan.current_plan.root.resume()
                sleep(3)
                self.assertEqual(0.3, self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position)

            code_node = CodeNode(pause_plan)
            sleep_node = CodeNode(node_sleep)
            robot_plan = SequentialPlan(self.context,  Plan(sleep_node, *self.context), MoveTorsoActionDescription([TorsoState.HIGH]))

            with simulated_robot:
                ParallelPlan(self.context,  Plan(code_node, self.context), robot_plan).perform()

            self.assertEqual(0.3, self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position)


class AlgebraTest(BulletWorldTestCase):

    def test_algebra(self):
        sp = SequentialPlan(self.context,
                            MoveTorsoActionDescription(None),
                            NavigateActionDescription(None),
                            MoveTorsoActionDescription(None))

        p = Parameterizer(sp)
        distribution = p.create_fully_factorized_distribution()

        conditions = []
        for state in TorsoState:
            v1 = p.get_variable("MoveTorsoAction_0.torso_state")
            v2 = p.get_variable("MoveTorsoAction_2.torso_state")
            se = SimpleEvent({v1: state, v2: state})
            conditions.append(se)

        condition = Event(*conditions)
        condition.fill_missing_variables(p.variables)

        navigate_condition = {
            p.get_variable("NavigateAction_1.target_location.pose.position.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.x"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.y"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.w"): 1
        }

        distribution, _ = distribution.conditional(navigate_condition)

        condition &= p.create_restrictions().as_composite_set()

        conditional, p_c = distribution.truncated(condition)

        for i in range(10):
            sample = conditional.sample(1)

            resolved = p.plan_from_sample(conditional, sample[0], self.world)
            with simulated_robot:
                resolved.perform()