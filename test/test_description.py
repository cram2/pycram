import pathlib

from bullet_world_testcase import BulletWorldTestCase


class DescriptionTest(BulletWorldTestCase):
    def test_is_root(self):
        self.assertTrue(self.robot.root_link.is_root)
        self.assertFalse(self.robot.links["base_link"].is_root)

    def test_orientation_as_list(self):
        base_link = self.robot.links["base_link"]
        self.assertEqual(base_link.orientation_as_list, base_link.pose.orientation_as_list())

    def test_pose_as_list(self):
        base_link = self.robot.links["base_link"]
        self.assertEqual(base_link.pose_as_list, base_link.pose.to_list())

    def test_joint_child_link(self):
        self.assertEqual(self.robot.get_joint_child_link("base_footprint_joint"), self.robot.get_link("base_link"))
        self.assertEqual(self.robot.get_joint_parent_link("base_footprint_joint"), self.robot.root_link)

    def test_generate_description_from_mesh(self):
        file_path = pathlib.Path(__file__).parent.resolve()
        self.assertTrue(self.milk.description.generate_description_from_file(str(file_path) + "/../resources/cached/milk.stl",
                                                                             "milk", ".stl"))

    def test_generate_description_from_description_file(self):
        file_path = pathlib.Path(__file__).parent.resolve()
        self.assertTrue(self.milk.description.generate_description_from_file(str(file_path) + "/../resources/cached/milk.urdf",
                                                                             "milk", ".urdf"))
