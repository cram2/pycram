from pycram.designators.actions.actions import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable
from pycram.pose import Pose
from pycram.process_module import simulated_robot
import pycram.task
from pycram.task import with_tree
import unittest
import anytree
from  bullet_world_testcase import BulletWorldTestCase
import pycram.plan_failures
from pycram.designators import object_designator, action_designator


class TaskTreeTestCase(BulletWorldTestCase):

    @with_tree
    def plan(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), "left", "front").perform()
            description.resolve().perform()

    def setUp(self):
        super().setUp()
        pycram.task.reset_tree()

    def test_tree_creation(self):
        """Test the creation and content of a task tree."""
        self.plan()
        # self.tearDownBulletWorld()
        tt = pycram.task.task_tree

        # print(anytree.RenderTree(tt))

        self.assertEqual(15, len(tt.root))
        self.assertEqual(10, len(tt.root.leaves))
        names = [node.code.function.__name__ for node in anytree.PreOrderIter(tt.root)]
        self.assertEqual(names, ["no_operation", "plan", "perform", "perform", "perform", "perform", "perform",
                                 "perform", "perform", "perform", "perform", "perform", "perform", "perform", "perform"]
                         )

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

        self.assertRaises(pycram.plan_failures.PlanFailure, failing_plan)

        tt = pycram.task.task_tree

        for node in anytree.PreOrderIter(tt.root):
            if node.code != pycram.task.NoOperation():
                self.assertEqual(node.status, pycram.task.TaskStatus.FAILED)
                self.assertEqual(pycram.plan_failures.PlanFailure, type(node.reason))

    def test_execution(self):
        self.plan()
        self.world.reset_bullet_world()
        tt = pycram.task.task_tree
        # self.setUpBulletWorld(False)
        with simulated_robot:
            [node.code.execute() for node in tt.root.leaves]

    def test_simulated_tree(self):
        with pycram.task.SimulatedTaskTree() as st:
            self.plan()
            tt = pycram.task.task_tree

            self.assertEqual(15, len(tt.root))
            self.assertEqual(10, len(tt.root.leaves))
            names = [node.code.function.__name__ for node in anytree.PreOrderIter(tt.root)]
            self.assertEqual(names, ["simulation", "plan", "perform", "perform", "perform", "perform", "perform",
                                     "perform", "perform", "perform", "perform", "perform", "perform", "perform",
                                     "perform"])

        self.assertEqual(len(pycram.task.task_tree), 1)


if __name__ == '__main__':
    unittest.main()


