from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.resolver.plans import Arms
import pycram.task
from pycram.task import with_tree
import unittest
import anytree
import test_bullet_world
import pycram.plan_failures


class TestTaskTree(test_bullet_world.BulletWorldTest):

    @with_tree
    def plan(self):
        with simulated_robot:
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

            location = LocationDesignator(CostmapLocation(target=self.milk, reachable_for=self.robot))
            pose = location.reference()
            ActionDesignator(
                NavigateAction(target_position=pose["position"], target_orientation=pose["orientation"])).perform()
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

    def setUp(self):
        super().setUp()
        pycram.task.reset_tree()

    def test_tree_creation(self):
        """Test the creation and content of a task tree."""
        self.plan()
        # self.tearDownBulletWorld()
        tt = pycram.task.task_tree

        self.assertEqual(5, len(tt.root))
        self.assertEqual(3, len(tt.root.leaves))
        names = [node.code.function.__name__ for node in anytree.PreOrderIter(tt.root)]
        self.assertEqual(names, ["no_operation", "plan", "park_arms", "navigate", "park_arms"])

        # check that all nodes succeeded
        for node in anytree.PreOrderIter(tt.root):
            if node.code != pycram.task.NoOperation():
                self.assertEqual(node.status, pycram.task.TaskStatus.SUCCEEDED)

    def test_exception(self):
        """Test the tree with failing plans."""

        @with_tree
        def failing_plan():
            raise pycram.plan_failures.PlanFailure("PlanFailure for UnitTesting")
        pycram.task.reset_tree()
        failing_plan()
        tt = pycram.task.task_tree
        for node in anytree.PreOrderIter(tt.root):
            if node.code != pycram.task.NoOperation():
                self.assertEqual(node.status, pycram.task.TaskStatus.FAILED)

    def test_execution(self):
        self.plan()
        self.world.reset_bullet_world()
        tt = pycram.task.task_tree
        # self.setUpBulletWorld(False)
        with simulated_robot:
            [node.code.execute() for node in tt.root.leaves]

    def test_to_json(self):
        """Test json serialization"""
        pass

    def test_simulated_tree(self):
        with pycram.task.SimulatedTaskTree() as st:
            self.plan()
            tt = pycram.task.task_tree

            self.assertEqual(5, len(tt.root))
            self.assertEqual(3, len(tt.root.leaves))
            names = [node.code.function.__name__ for node in anytree.PreOrderIter(tt.root)]
            self.assertEqual(names, ["simulation", "plan", "park_arms", "navigate", "park_arms"])

        self.assertEqual(len(pycram.task.task_tree), 1)


if __name__ == '__main__':
    unittest.main()


