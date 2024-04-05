from bullet_world_testcase import BulletWorldTestCase
from pycram.datastructures.dataclasses import Color


class TestLinks(BulletWorldTestCase):

    def test_add_constraint(self):
        milk_link = self.milk.root_link
        robot_link = self.robot.root_link
        robot_link.add_fixed_constraint_with_link(milk_link)
        self.assertTrue(milk_link in robot_link.constraint_ids)
        self.assertTrue(robot_link in milk_link.constraint_ids)

    def test_remove_constraint(self):
        milk_link = self.milk.root_link
        robot_link = self.robot.root_link
        robot_link.add_fixed_constraint_with_link(milk_link)
        robot_link.remove_constraint_with_link(milk_link)
        self.assertTrue(milk_link not in robot_link.constraint_ids)
        self.assertTrue(robot_link not in milk_link.constraint_ids)

    def test_transform_link(self):
        milk_link = self.milk.root_link
        robot_link = self.robot.root_link
        milk_to_robot = milk_link.get_transform_to_link(robot_link)
        robot_to_milk = robot_link.get_transform_to_link(milk_link)
        self.assertAlmostEqual(robot_to_milk, milk_to_robot.invert())

    def test_set_color(self):
        link = self.robot.get_link('base_link')
        link.color = Color(1, 0, 0, 1)
        self.assertEqual(link.color.get_rgba(), [1, 0, 0, 1])


