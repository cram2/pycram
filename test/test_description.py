import os.path
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
        cache_path = self.world.cache_manager.cache_dir
        cache_path = os.path.join(cache_path, f"{self.milk.description.name}.urdf")
        self.milk.description.generate_from_mesh_file(str(file_path) + "/../resources/milk.stl", "milk", cache_path)
        self.assertTrue(self.world.cache_manager.is_cached(f"{self.milk.name}", self.milk.description))

    def test_generate_description_from_description_file(self):
        file_path = pathlib.Path(__file__).parent.resolve()
        file_extension = self.robot.description.get_file_extension()
        pr2_path = str(file_path) + f"/../resources/robots/{self.robot.description.name}{file_extension}"
        cache_path = self.world.cache_manager.cache_dir
        cache_path = os.path.join(cache_path, f"{self.robot.description.name}.urdf")
        self.robot.description.generate_from_description_file(pr2_path, cache_path)
        self.assertTrue(self.world.cache_manager.is_cached(self.robot.name, self.robot.description))
