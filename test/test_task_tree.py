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


class TestTaskTree(unittest.TestCase):

    def setUp(self) -> None:
        self.world = BulletWorld()
        # self.world = BulletWorld("DIRECT")
        self.pr2 = Object("pr2", "robot", "pr2.urdf")
        self.kitchen = Object("kitchen", "environment", "kitchen.urdf")

        self.milk = Object("milk", "milk", "milk.stl", position=[1.3, 1, 0.9])
        self.cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=[1.3, 0.7, 0.95])

        self.milk_desig = ObjectDesignator(ObjectDesignatorDescription(name="milk", type="milk"))
        self.cereal_desig = ObjectDesignator(ObjectDesignatorDescription(name="cereal", type="cereal"))

    def reset(self):
        self.milk.detach(self.pr2)
        self.pr2.set_position_and_orientation([0, 0, 0], [0, 0, 0, 1])
        self.milk.set_position([1.3, 1, 0.9])
        self.cereal.set_position([1.3, 0.7, 0.95])
        ActionDesignator(ParkArmsAction(Arms.BOTH)).perform()

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
        self.plan()
        result = []

        nodes = collections.deque([pycram.task.TASK_TREE])
        print(anytree.RenderTree(pycram.task.task_tree.root, style=anytree.render.AsciiStyle()))

        while nodes:
            current_node = nodes.pop()
            result.append(current_node.important_information())
            nodes.extend(current_node.children)
        print(result)
        self.world.exit()


if __name__ == '__main__':
    unittest.main()


