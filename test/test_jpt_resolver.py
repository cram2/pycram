import os
import time
import unittest

import jpt

from pycram.bullet_world import BulletWorld, Object
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.resolver.location.jpt_location import JPTCostmapLocation
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description


class JPTResolverTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    robot: Object
    model: jpt.JPT

    @classmethod
    def setUpClass(cls) -> None:
        cls.model = jpt.JPT.load(os.path.join(os.path.expanduser("~"), "Documents", "grasping.jpt"))
        cls.world = BulletWorld("GUI")
        cls.milk = Object("milk", "milk", "milk.stl", position=[3, 3, 0.75])
        cls.robot = Object(robot_description.i.name, "pr2", robot_description.i.name + ".urdf")
        ProcessModule.execution_delay = False

    def test_costmap(self):
        cml = JPTCostmapLocation(self.milk, reachable_for=self.robot, model=self.model)
        sample = next(iter(cml))
        print(sample)
        cml.visualize()

        with simulated_robot:
            action_designator.NavigateAction.Action(sample.pose).perform()
            action_designator.MoveTorsoAction.Action(sample.torso_height).perform()
            time.sleep(5)
            action_designator.PickUpAction.Action(
                object_designator.ObjectDesignatorDescription(types=["milk"]).resolve(),
                arm=sample.reachable_arm, grasp=sample.grasp).perform()
            time.sleep(5)
    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.exit()


if __name__ == '__main__':
    unittest.main()
