import numpy as np
from owlready2 import Imp

from pycram.tf_transformations import quaternion_from_euler

from pycram.datastructures.dataclasses import Color, BoundingBox as BB
from pycram.datastructures.pose import PoseStamped
from pycram.testing import BulletWorldTestCase
from pycrap.ontologies import Room, Food


class TestLinks(BulletWorldTestCase):

    def tearDown(self):
        super().tearDown()
        self.world.reset_concepts()

    def test_automatic_containment_detection(self):

        self.world.update_containment_for([self.milk])

        self.assertFalse(self.kitchen.contains_body(self.kitchen.links["iai_fridge_main"]))
        self.assertFalse(self.kitchen.contains_body(self.milk))

        self.milk.set_position(self.kitchen.links["iai_fridge_main"].position)

        self.world.update_containment_for([self.milk])

        self.assertTrue(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))

    def test_automatic_containment_detection_from_parts(self):

        self.assertFalse(self.kitchen.contains_body(self.kitchen.links["iai_fridge_main"]))
        self.assertFalse(self.kitchen.contains_body(self.milk))

        self.world.update_containment_for([self.kitchen])
        self.world.update_containment_for([self.milk])

        self.milk.set_position(self.kitchen.links["iai_fridge_main"].position)

        self.world.update_containment_for([self.milk])

        self.assertTrue(self.kitchen.contains_body(self.milk))
        self.assertTrue(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))

    def test_contains_body(self):

        self.assertFalse(self.kitchen.contains_body(self.kitchen.links["iai_fridge_main"]))
        self.assertFalse(self.kitchen.contains_body(self.milk))

        self.kitchen.contained_bodies = [self.kitchen.links["iai_fridge_main"]]
        self.assertTrue(self.kitchen.contains_body(self.kitchen.links["iai_fridge_main"]))
        self.assertFalse(self.kitchen.contains_body(self.milk))

        self.kitchen.links["iai_fridge_main"].contained_bodies = [self.milk]

        # Kitchen should contain cabinet and drawer (reasoned)
        self.assertTrue(self.kitchen.contains_body(self.kitchen.links["iai_fridge_main"]))
        self.assertTrue(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))
        self.assertTrue(self.kitchen.contains_body(self.milk))
        self.assertEqual(len(self.kitchen.contained_bodies), 2)

    def test_get_convex_hull(self):
        self.milk.set_orientation(quaternion_from_euler(0, np.pi / 4, 0))
        hull = self.milk.root_link.get_convex_hull()
        self.assertIsNotNone(hull)
        self.assertTrue(len(hull.vertices) > 0)
        self.assertTrue(len(hull.faces) > 0)
        plot = False
        if plot:
            BB.plot_3d_points([hull.vertices])

    def test_rotated_bounding_box(self):
        self.milk.set_pose(PoseSteamped.from_list([1, 1, 1], quaternion_from_euler(np.pi / 4, 0, 0)))
        aabb = self.milk.get_axis_aligned_bounding_box()
        aabb_points = np.array(aabb.get_points_list())
        rbb = self.milk.get_rotated_bounding_box()
        rot_points = np.array(rbb.get_points_list())
        plot = False
        if plot:
            BB.plot_3d_points([aabb_points, rot_points])

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
