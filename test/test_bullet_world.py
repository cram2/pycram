import unittest

import rospkg

from bullet_world_testcase import BulletWorldTestCase
from pycram.pose import Pose
import numpy as np
from pycram.bullet_world import fix_missing_inertial
import xml.etree.ElementTree as ET


class BulletWorldTest(BulletWorldTestCase):

    def test_object_movement(self):
        self.milk.set_position(Pose([0, 1, 1]))
        self.assertEqual(self.milk.get_pose().position_as_list(), [0, 1, 1])

    def test_robot_orientation(self):
        self.robot.set_pose(Pose([0, 1, 1]))
        head_position = self.robot.get_link_position('head_pan_link').z
        #self.robot.set_orientation(list(2*tf.transformations.quaternion_from_euler(0, 0, np.pi, axes="sxyz")))
        self.robot.set_orientation(Pose(orientation=[0, 0, 1, 1]))
        self.assertEqual(self.robot.get_link_position('head_pan_link').z, head_position)


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

