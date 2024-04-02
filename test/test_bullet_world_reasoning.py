import time

from bullet_world_testcase import BulletWorldTestCase
import pycram.bullet_world_reasoning as btr
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description


class TestCaseBulletWorldReasoning(BulletWorldTestCase):

    def test_contact(self):
        self.milk.set_pose(Pose([1, 1, 1]))
        self.cereal.set_pose(Pose([1, 1, 1]))
        time.sleep(1)
        self.assertTrue(btr.contact(self.milk, self.cereal))

    def test_visible(self):
        self.milk.set_pose(Pose([1.5, 0, 1.2]))
        self.robot.set_pose(Pose())
        time.sleep(1)
        self.assertTrue(btr.visible(self.milk, self.robot.get_link_pose(robot_description.get_camera_frame()),
                                    robot_description.front_facing_axis))

    def test_occluding(self):
        self.milk.set_pose(Pose([3, 0, 1.2]))
        self.robot.set_pose(Pose())
        self.assertTrue(btr.occluding(self.milk, self.robot.get_link_pose(robot_description.get_camera_frame()),
                                      robot_description.front_facing_axis) != [])

    def test_reachable(self):
        self.robot.set_pose(Pose())
        self.assertTrue(btr.reachable(Pose([0.5, -0.7, 1]), self.robot, robot_description.get_tool_frame("right")))
        self.assertFalse(btr.reachable(Pose([2, 2, 1]), self.robot, robot_description.get_tool_frame("right")))

    def test_blocking(self):
        self.milk.set_pose(Pose([0.5, -0.7, 1]))
        self.robot.set_pose(Pose())
        time.sleep(2)
        self.assertTrue(btr.blocking(Pose([0.5, -0.7, 1]), self.robot, robot_description.get_tool_frame("right")) != [])

    def test_supporting(self):
        self.milk.set_pose(Pose([1.3, 0, 0.9]))
        self.assertTrue(btr.supporting(self.kitchen, self.milk))
