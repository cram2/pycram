from pycram.designators.action_designator import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms, Grasp, GripperState
from pycram.process_module import simulated_robot
import pycram.tasktree
from pycram.tasktree import with_tree
import unittest
import anytree
from bullet_world_testcase import BulletWorldTestCase
import pycram.plan_failures
from pycram.designators import object_designator, action_designator


class TaskTreeTestCase(BulletWorldTestCase):

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()

    def setUp(self):
        super().setUp()
        pycram.tasktree.reset_tree()

    def test_tree_creation(self):
        """Test the creation and content of a task tree."""
        self.plan()
        # self.tearDownBulletWorld()
        tt = pycram.tasktree.task_tree

        self.assertEqual(15, len(tt.root))
        self.assertEqual(10, len(tt.root.leaves))

        # check that all nodes succeeded
        for node in anytree.PreOrderIter(tt.root):
            if not isinstance(node.action, pycram.tasktree.NoOperation):
                self.assertEqual(node.status, pycram.tasktree.TaskStatus.SUCCEEDED)

    def test_exception(self):
        """Test the tree with failing plans."""

        @with_tree
        def failing_plan():
            raise pycram.plan_failures.PlanFailure("PlanFailure for UnitTesting")

        pycram.tasktree.reset_tree()

        self.assertRaises(pycram.plan_failures.PlanFailure, failing_plan)

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

            self.assertEqual(15, len(tt.root))
            self.assertEqual(10, len(tt.root.leaves))

        self.assertEqual(len(pycram.tasktree.task_tree), 1)

    def test_to_sql(self):
        self.plan()
        tt = pycram.tasktree.task_tree
        result = tt.root.to_sql()
        self.assertIsNotNone(result)


if __name__ == '__main__':
    unittest.main()


