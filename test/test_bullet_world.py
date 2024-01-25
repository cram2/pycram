import unittest

import rospkg

from bullet_world_testcase import BulletWorldTestCase
from pycram.pose import Pose
from pycram.world import fix_missing_inertial
import xml.etree.ElementTree as ET


class BulletWorldTest(BulletWorldTestCase):

    def test_object_movement(self):
        self.milk.set_position(Pose([0, 1, 1]))
        self.assertEqual(self.milk.get_position_as_list(), [0, 1, 1])

    def test_robot_orientation(self):
        self.robot.set_pose(Pose([0, 1, 1]))
        head_position = self.robot.links['head_pan_link'].position.z
        self.robot.set_orientation(Pose(orientation=[0, 0, 1, 1]))
        self.assertEqual(self.robot.links['head_pan_link'].position.z, head_position)

    def test_save_and_restore_state(self):
        self.robot.attach(self.milk)
        state_id = self.world.save_state()
        robot_link = self.robot.get_root_link()
        milk_link = self.milk.get_root_link()
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(cid == self.robot.attachments[self.milk].constraint_id)
        self.world.remove_constraint(cid)
        self.world.restore_state(state_id)
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(milk_link in robot_link.constraint_ids)
        self.assertTrue(self.milk in self.robot.attachments)
        self.assertTrue(cid == self.robot.attachments[self.milk].constraint_id)


class XMLTester(unittest.TestCase):

    def setUp(self) -> None:
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pycram') + '/resources/' + 'pr2.urdf'
        with open(filename, "r") as f:
            self.urdf_string = f.read()

    def test_inertial(self):
        result = fix_missing_inertial(self.urdf_string)
        resulting_tree = ET.ElementTree(ET.fromstring(result))
        for element in resulting_tree.iter("link"):
            self.assertTrue(len([*element.iter("inertial")]) > 0)
