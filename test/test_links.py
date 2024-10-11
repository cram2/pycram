import numpy as np
from matplotlib import pyplot as plt
from tf.transformations import quaternion_from_euler

from bullet_world_testcase import BulletWorldTestCase
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.pose import Pose


class TestLinks(BulletWorldTestCase):

    def test_rotated_bounding_box(self):
        self.milk.set_pose(Pose([1, 1, 1], quaternion_from_euler(np.pi/4, 0, 0).tolist()))
        aabb = self.milk.get_axis_aligned_bounding_box()
        aabb_points = np.array(aabb.get_corners())
        rbb = self.milk.get_rotated_bounding_box()
        rot_points = np.array(rbb.get_corners())
        plot = False
        if plot:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')

            ax.scatter(rot_points[:, 0], rot_points[:, 1], rot_points[:, 2], c='r', marker='o')
            ax.scatter(aabb_points[:, 0], aabb_points[:, 1], aabb_points[:, 2], c='b', marker='o')

            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            plt.xlim(0, 2)
            plt.ylim(0, 2)
            ax.set_zlim(0, 2)

            plt.show()

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


