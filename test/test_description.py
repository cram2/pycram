import os.path
import pathlib
from copy import deepcopy

from pycram.testing import BulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Food


class DescriptionTest(BulletWorldTestCase):
    def test_is_root(self):
        self.assertTrue(self.robot.root_link.is_root)
        self.assertFalse(self.robot.links["base_link"].is_root)

    def test_orientation_as_list(self):
        base_link = self.robot.links["base_link"]
        self.assertEqual(base_link.orientation.to_list(), base_link.pose.orientation.to_list()())

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

    def test_merge_descriptions(self):
        cereal_2 = Object("cereal_2", Food, self.cereal.path)
        milk_link_name = self.milk.description.get_root()
        milk_joints_num = len(self.milk.description.joints)
        cereal_joints_num = len(cereal_2.description.joints)
        milk_link_num = len(self.milk.description.links)
        cereal_link_num = len(cereal_2.description.links)
        original_cereal_description = deepcopy(cereal_2.description)
        for in_place in [False, True]:
            for new_description_path in [None, "cereal_milk"]:
                description = cereal_2.description.merge_description(self.milk.description, in_place=in_place,
                                                                     new_description_file=new_description_path)
                if in_place:
                    self.assertTrue(description is cereal_2.description)
                    self.assertEqual(len(cereal_2.description.joints), milk_joints_num + cereal_joints_num + 1)
                    self.assertEqual(len(cereal_2.description.links), milk_link_num + cereal_link_num)
                    self.assertTrue(f"{cereal_2.root_link.name}_{self.milk.root_link.name}_joint"
                                    in cereal_2.description.joint_map.keys())
                    self.assertTrue(milk_link_name not in cereal_2.description.link_map.keys())
                    self.assertTrue(f"{self.milk.description.name}_{milk_link_name}" in cereal_2.description.link_map.keys())
                else:
                    self.assertFalse(cereal_2.description is description)
                    self.assertEqual(len(description.joints), milk_joints_num + cereal_joints_num + 1)
                    self.assertEqual(len(description.links), milk_link_num + cereal_link_num)
                    self.assertTrue(f"{cereal_2.root_link.name}_{self.milk.root_link.name}_joint"
                                    in description.joint_map.keys())
                    self.assertTrue(milk_link_name not in description.link_map.keys())
                    self.assertTrue(
                        f"{self.milk.description.name}_{milk_link_name}" in description.link_map.keys())
                if in_place:
                    cereal_2.description = original_cereal_description
        cereal_2.remove()
        self.world.cache_manager.clear_cache()
