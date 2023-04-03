from pycram.designators.motion_designator import *
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot
from pycram.resolver.plans import Arms
import pycram.task
from pycram.task import with_tree
import collections
import unittest
import anytree
import copy


class TestTaskTree(unittest.TestCase):

    def setUpBulletWorld(self, reset_task_tree=True) -> None:
        if reset_task_tree:
            pycram.task.reset_tree()
        self.world = BulletWorld()
        # self.world = BulletWorld("DIRECT")
        self.pr2 = Object("pr2", "robot", "pr2.urdf")
        self.kitchen = Object("kitchen", "environment", "kitchen.urdf")

        self.milk = Object("milk", "milk", "milk.stl", position=[1.3, 1, 0.9])
        self.cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=[1.3, 0.7, 0.95])

        self.milk_desig = ObjectDesignator(ObjectDesignatorDescription(name="milk", type="milk"))
        self.cereal_desig = ObjectDesignator(ObjectDesignatorDescription(name="cereal", type="cereal"))

    @with_tree
    def plan(self):
        with simulated_robot:
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

            location = LocationDesignator(CostmapLocationDesignatorDescription(target=self.milk,
                                                                               reachable_for=self.pr2))
            pose = location.reference()
            ActionDesignator(
                NavigateAction(target_position=pose["position"], target_orientation=pose["orientation"])).perform()
            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()
            return
            ActionDesignator(PickUpAction(object_designator=self.milk_desig, arm="right", grasp="front")).perform()

            ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

    def test_tree_creation(self):
        """Test the creation and content of a task tree."""
        self.setUpBulletWorld()
        self.plan()
        self.tearDownBulletWorld()
        tt = pycram.task.task_tree
        # print(anytree.RenderTree(tt.root, anytree.render.AsciiStyle()))

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
            raise ValueError("Exception for UnitTesting")
        pycram.task.reset_tree()
        failing_plan()
        tt = pycram.task.task_tree
        for node in anytree.PreOrderIter(tt.root):
            if node.code != pycram.task.NoOperation():
                self.assertEqual(node.status, pycram.task.TaskStatus.FAILED)

    def test_execution(self):
        self.setUpBulletWorld()
        self.plan()
        self.world.reset_bullet_world()
        # self.tearDownBulletWorld()
        tt = pycram.task.task_tree
        # self.setUpBulletWorld(False)
        with simulated_robot:
            [node.code.execute() for node in tt.root.leaves]

        self.tearDownBulletWorld()

    def test_simulated_tree(self):
        self.setUpBulletWorld()
        with pycram.task.SimulatedTaskTree() as st:
            self.plan()
            tt = pycram.task.task_tree

            self.assertEqual(5, len(tt.root))
            self.assertEqual(3, len(tt.root.leaves))
            names = [node.code.function.__name__ for node in anytree.PreOrderIter(tt.root)]
            self.assertEqual(names, ["simulation", "plan", "park_arms", "navigate", "park_arms"])

        self.assertEqual(len(pycram.task.task_tree), 1)
        self.tearDownBulletWorld()

    def tearDownBulletWorld(self) -> None:
        self.world.exit()


if __name__ == '__main__':
    unittest.main()


