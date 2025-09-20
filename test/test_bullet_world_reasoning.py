import time

import pycram.world_reasoning as btr
from pycram.multirobot import RobotManager
from pycram.testing import BulletWorldTestCase
from pycram.datastructures.pose import PoseStamped
from pycram.robot_description import RobotDescription


class TestCaseBulletWorldReasoning(BulletWorldTestCase):

    def test_contact(self):
        self.milk.set_pose(PoseStamped.from_list([1, 1, 1]))
        self.cereal.set_pose(PoseStamped.from_list([1, 1, 1]))
        time.sleep(1)
        self.assertTrue(btr.contact(self.milk, self.cereal))

    def test_visible(self):
        self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
        self.robot.set_pose(PoseStamped.from_list())
        time.sleep(1)
        camera_link = RobotManager.get_robot_description().get_camera_link()
        self.world.add_vis_axis(self.robot.get_link_pose(camera_link))
        self.assertTrue(btr.visible(self.milk, self.robot.get_link_pose(camera_link),
                                    RobotManager.get_robot_description().get_default_camera().front_facing_axis))

    def test_occluding(self):
        self.milk.set_pose(PoseStamped.from_list([3, 0, 1.2]))
        self.robot.set_pose(PoseStamped())
        self.assertTrue(btr.occluding(self.milk, self.robot.get_link_pose(
            RobotManager.get_robot_description().get_camera_link()),
                                      RobotManager.get_robot_description().get_default_camera().front_facing_axis) != [])

    def test_reachable(self):
        self.robot.set_pose(PoseStamped())
        time.sleep(1)
        self.assertTrue(btr.reachable(PoseStamped.from_list([0.5, -0.5, 0.5]), self.robot,
                                      RobotManager.get_robot_description().kinematic_chains[
                                          "right"].get_tool_frame()))
        self.assertFalse(btr.reachable(PoseStamped.from_list([2, 2, 1]), self.robot,
                                       RobotManager.get_robot_description().kinematic_chains[
                                           "right"].get_tool_frame()))


    def test_blocking(self):
        self.milk.set_pose(PoseStamped.from_list([0.5, -0.5, 0.7]))
        self.robot.set_pose(PoseStamped())
        time.sleep(2)
        blocking = btr.blocking(PoseStamped.from_list([0.5, -0.5, 0.7]), self.robot,
                                RobotManager.get_robot_description().kinematic_chains["right"])
        self.assertTrue(blocking != [])

    def test_supporting(self):
        self.milk.set_pose(PoseStamped.from_list([1.3, 0, 0.9]))
        self.assertTrue(btr.supporting(self.kitchen, self.milk))
