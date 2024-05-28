import numpy as np

from bullet_world_testcase import BulletWorldTestCase

from pycram.datastructures.enums import JointType, ObjectType
from pycram.datastructures.pose import Pose
from pycram.datastructures.dataclasses import Color
from pycram.world_concepts.world_object import Object

from geometry_msgs.msg import Point, Quaternion
import pathlib

class TestObject(BulletWorldTestCase):

    def test_wrong_object_description_path(self):
        with self.assertRaises(FileNotFoundError):
            milk = Object("milk_not_found", ObjectType.MILK, "wrong_path.sk")

    def test_malformed_object_description(self):
        file_path = pathlib.Path(__file__).parent.resolve()
        malformed_file = str(file_path) + "/../resources/cached/malformed_description.urdf"
        with open(malformed_file, "w") as file:
            file.write("malformed")
        with self.assertRaises(Exception):
            Object("milk2", ObjectType.MILK, malformed_file)

    def test_move_base_to_origin_pose(self):
        self.milk.set_position(Point(1, 2, 3), base=False)
        self.milk.move_base_to_origin_pose()
        self.assertEqual(self.milk.get_base_position_as_list(), [1, 2, 3])

    def test_set_position_as_point(self):
        self.milk.set_position(Point(1, 2, 3))
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_uni_direction_attachment(self):
        self.milk.attach(self.cereal, bidirectional=False)

        milk_position = self.milk.get_position_as_list()
        cereal_position = self.cereal.get_position_as_list()

        def move_milk_and_assert_cereal_moved():
            milk_position[0] += 1
            cereal_position[0] += 1
            self.milk.set_position(milk_position)

            new_cereal_position = self.cereal.get_position_as_list()
            self.assertEqual(new_cereal_position, cereal_position)

        def move_cereal_and_assert_milk_not_moved():
            cereal_position[0] += 1
            self.cereal.set_position(cereal_position)

            new_milk_position = self.milk.get_position_as_list()
            self.assertEqual(new_milk_position, milk_position)

        # move twice to test updated attachment at the new cereal position
        for _ in range(2):
            move_milk_and_assert_cereal_moved()
            move_cereal_and_assert_milk_not_moved()

    def test_setting_wrong_position_type(self):
        with self.assertRaises(TypeError):
            self.milk.set_position(np.array([1, 2, 3]))

        with self.assertRaises(TypeError):
            self.milk.get_pose().position = np.array([1, 2, 3])

    def test_set_orientation_as_list(self):
        self.milk.set_orientation([1, 0, 0, 0])
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_orientation_as_quaternion(self):
        self.milk.set_orientation(Quaternion(*[1, 0, 0, 0]))
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_orientation_as_ndarray(self):
        self.milk.set_orientation(np.array([1, 0, 0, 0]))
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_wrong_orientation_type(self):
        with self.assertRaises(TypeError):
            self.milk.set_orientation(1)

    def test_set_position_as_pose(self):
        self.milk.set_position(Pose([1, 2, 3]))
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_set_position_as_list(self):
        self.milk.set_position([1, 2, 3])
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_get_joint_axis(self):
        self.assertEqual(self.robot.get_joint_axis("head_pan_joint"), Point(0.0, 0.0, 1.0))

    def test_get_joint_type(self):
        self.assertEqual(self.robot.get_joint_type("head_pan_joint"), JointType.REVOLUTE)

    def test_get_joint_lower_limit(self):
        self.assertEqual(self.robot.get_joint_lower_limit("head_pan_joint"), -3.007)

    def test_get_joint_upper_limit(self):
        self.assertEqual(self.robot.get_joint_upper_limit("head_pan_joint"), 3.007)

    def test_get_joint_damping(self):
        self.assertEqual(self.robot.get_joint_damping("head_pan_joint"), 0.5)

    def test_save_state(self):
        self.robot.attach(self.milk)
        self.robot.save_state(1)
        self.assertEqual(self.robot.saved_states[1].attachments, self.robot.attachments)
        self.assertTrue(self.milk in self.robot.saved_states[1].attachments)
        for link in self.robot.links.values():
            self.assertEqual(link.current_state, link.saved_states[1])

    def test_restore_state(self):
        self.robot.attach(self.milk)
        self.robot.save_state(1)
        self.milk.save_state(1)
        self.assertTrue(self.milk in self.robot.attachments)
        self.robot.detach(self.milk)
        self.assertTrue(self.milk not in self.robot.attachments)
        self.robot.restore_state(1)
        self.milk.restore_state(1)
        self.assertTrue(self.milk in self.robot.attachments)
        for link in self.robot.links.values():
            curr_state = link.current_state
            saved_state = link.saved_states[1]
            self.assertEqual(curr_state, saved_state)
            self.assertEqual(curr_state.constraint_ids, saved_state.constraint_ids)

    def test_get_link_by_id(self):
        self.assertEqual(self.robot.get_link_by_id(-1), self.robot.root_link)

    def test_find_joint_above_link(self):
        self.assertEqual(self.robot.find_joint_above_link("head_pan_link", JointType.REVOLUTE), "head_pan_joint")

    def test_wrong_joint_type_for_joint_above_link(self):
        container_joint = self.robot.find_joint_above_link("head_pan_link", JointType.CONTINUOUS)
        self.assertTrue(container_joint is None)

    def test_contact_points_simulated(self):
        self.milk.set_position([0, 0, 100])
        contact_points = self.milk.contact_points_simulated()
        self.assertFalse(contact_points)
        self.milk.set_position(self.cereal.get_position_as_list(), base=True)
        contact_points = self.milk.contact_points_simulated()
        self.assertTrue(contact_points)

    def test_set_color(self):
        self.milk.set_color(Color(1, 0, 0, 1))
        self.assertEqual(self.milk.get_color(), Color(1, 0, 0, 1))
        self.robot.set_color(Color(0, 1, 0, 1))
        for color in self.robot.get_color().values():
            self.assertEqual(color, Color(0, 1, 0, 1))

    def test_object_equal(self):
        milk2 = Object("milk2", ObjectType.MILK, "milk.stl")
        self.assertNotEqual(self.milk, milk2)
        self.assertEqual(self.milk, self.milk)
        self.assertNotEqual(self.milk, self.cereal)
        self.assertNotEqual(self.milk, self.world)
