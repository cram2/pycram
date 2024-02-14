import time
import unittest

import rospkg

from bullet_world_testcase import BulletWorldTestCase
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.enums import ObjectType
from pycram.world_object import Object
import xml.etree.ElementTree as ET
from pycram.urdf_interface import ObjectDescription

fix_missing_inertial = ObjectDescription.fix_missing_inertial


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
        robot_link = self.robot.root_link
        milk_link = self.milk.root_link
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(cid == self.robot.attachments[self.milk].id)
        self.world.remove_constraint(cid)
        self.world.restore_state(state_id)
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(milk_link in robot_link.constraint_ids)
        self.assertTrue(self.milk in self.robot.attachments)
        self.assertTrue(cid == self.robot.attachments[self.milk].id)

    def test_remove_object(self):
        time.sleep(2)
        milk_id = self.milk.id
        self.assertTrue(milk_id in [obj.id for obj in self.world.objects])
        self.world.remove_object(self.milk)
        self.assertTrue(milk_id not in [obj.id for obj in self.world.objects])
        BulletWorldTest.milk = Object("milk", ObjectType.MILK, "milk.stl", ObjectDescription, pose=Pose([1.3, 1, 0.9]))

    def test_get_joint_position(self):
        self.assertEqual(self.robot.get_joint_position("head_pan_joint"), 0.0)

    def test_get_object_contact_points(self):
        self.assertEqual(len(self.robot.contact_points()), 0)
        self.milk.set_position(self.robot.get_position())
        self.assertTrue(len(self.robot.contact_points()) > 0)

    def test_step_simulation(self):
        # TODO: kitchen explodes when stepping simulation, fix this
        time.sleep(2)
        self.world.remove_object(self.kitchen)
        self.milk.set_position(Pose([0, 0, 2]))
        self.world.simulate(1)
        self.assertTrue(self.milk.get_position().z < 2)

    def test_set_real_time_simulation(self):
        # self.world.remove_object(self.kitchen)
        self.milk.set_position(Pose([100, 0, 2]))
        curr_time = time.time()
        self.world.simulate(0.5, real_time=True)
        time_elapsed = time.time() - curr_time
        self.assertAlmostEqual(time_elapsed, 0.5, delta=0.1)

    def test_create_vis_axis(self):
        self.world.add_vis_axis(self.robot.links[robot_description.get_camera_frame()].pose, 1)
        self.assertTrue(len(self.world.vis_axis) == 1)
        self.world.remove_vis_axis()
        self.assertTrue(len(self.world.vis_axis) == 0)


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
