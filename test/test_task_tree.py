from pycram.datastructures.pose import GraspDescription
from pycram.designators.action_designator import MoveTorsoAction, PickUpAction, \
    NavigateAction
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import Arms, Grasp, GripperState, TorsoState
from pycram.process_module import simulated_robot
import pycram.tasktree
from pycram.tasktree import with_tree
import unittest
import anytree
from pycram.testing import BulletWorldTestCase
import pycram.failures
from pycram.designators import object_designator, action_designator
from pycram.designators.action_designator import MoveTorsoActionDescription


class TaskTreeTestCase(BulletWorldTestCase):

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceActionDescription(object_description, [PoseSteamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.resolve().object_designator.name, "milk")
        with simulated_robot:
            NavigateAction(PoseSteamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
            grasp_description = GraspDescription(Grasp.FRONT, None, False)
            PickUpAction(object_description.resolve(), Arms.LEFT, grasp_description, 0.03).perform()
            description.resolve().perform()

    def test_tree_creation(self):
        """Test the creation and content of a task tree."""
        self.plan()
        # self.tearDownBulletWorld()
        tt = pycram.tasktree.task_tree

        self.assertEqual(16, len(tt.root))
        self.assertEqual(10, len(tt.root.leaves))

        # check that all nodes succeeded
        for node in anytree.PreOrderIter(tt.root):
            if not isinstance(node.action, pycram.tasktree.NoOperation):
                self.assertEqual(node.status, pycram.tasktree.TaskStatus.SUCCEEDED)

    def test_exception(self):
        """Test the tree with failing plans."""

        @with_tree
        def failing_plan():
            raise pycram.failures.PlanFailure("PlanFailure for UnitTesting")

        pycram.tasktree.task_tree.reset_tree()

        self.assertRaises(pycram.failures.PlanFailure, failing_plan)

        tt = pycram.tasktree.task_tree

        for node in anytree.PreOrderIter(tt.root):
            if not isinstance(node.action, pycram.tasktree.NoOperation):
                self.assertEqual(node.status, pycram.tasktree.TaskStatus.FAILED)
                self.assertEqual(pycram.plan_failures.PlanFailure, type(node.reason))

    def test_execution(self):
        self.plan()
        self.world.reset_world()
        tt = pycram.tasktree.task_tree
        # self.setUpBulletWorld(False)
        with simulated_robot:
            [node.action.perform() for node in tt.root.leaves]

    def test_simulated_tree(self):
        with pycram.tasktree.SimulatedTaskTree() as st:
            self.plan()
            tt = pycram.tasktree.task_tree

            self.assertEqual(16, len(tt.root))
            self.assertEqual(10, len(tt.root.leaves))

        self.assertEqual(len(pycram.tasktree.task_tree), 1)

    def test_to_sql(self):
        self.plan()
        tt = pycram.tasktree.task_tree
        result = tt.root.to_sql()
        self.assertIsNotNone(result)

    def test_task_tree_singleton(self):
        # Instantiate one TaskTree object
        tree1 = pycram.tasktree.TaskTree()

        # Fill the tree
        self.plan()

        # Instantiate another TaskTree object
        tree2 = pycram.tasktree.TaskTree()

        # Check if both instances point to the same object and contain the same number of elements
        self.assertEqual(len(tree1.root), len(tree2.root))
        self.assertIs(tree1, tree2)

if __name__ == '__main__':
    unittest.main()


