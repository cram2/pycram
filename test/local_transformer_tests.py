import unittest
import rospy
from time import sleep

rospy.init_node('pycram')

from src.pycram.bullet_world import BulletWorld, Object
from src.pycram.local_transformer import LocalTransformer, LocalTransformerFreqPublisher
from pycram.robot_descriptions import robot_description

world = None
robot = None
ltf = LocalTransformer("map", "odom", "projection", "iai_kitchen")
freq_ltf = LocalTransformerFreqPublisher()

def park_arms():
    """
    Defines the joint poses for the parking positions of the arms of Boxy and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    global robot
    for joint, pose in robot_description.get_static_joint_chain("right", "park").items():
        robot.set_joint_state(joint, pose)
    for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
        robot.set_joint_state(joint, pose)


def clear():
    global ltf, freq_ltf
    # Stop the publisher threads and remove the local_transformer
    freq_ltf.stop_publishing()
    ltf = None
    freq_ltf = None


def setup():
    # Reset Robot Pose and Joint Values
    global robot, ltf, freq_ltf
    robot.set_position_and_orientation([0, 0, 0], [0, 0, 0, 1])
    for joint in robot.joints:
        robot.set_joint_state(joint, 0)
    # Reinitializes the local transformer and start publisher threads
    # Frame names of the map and odom frame
    map_frame = "map"
    odom_frame = "odom"
    # Namespaces
    projection_namespace = "projection"
    kitchen_namespace = "iai_kitchen"
    # Initializing Local Transformer using ROS data types
    ltf = LocalTransformer(map_frame, odom_frame, projection_namespace, kitchen_namespace)
    freq_ltf = LocalTransformerFreqPublisher()
    sleep(1)
    ltf.update_from_btr()




class LocalTransformerTests(unittest.TestCase):

    def assertAlmostEqualList(self, a_list, b_list, decimal_places=5):
        self.assertTrue(map(lambda a, b: self.assertAlmostEquals(a, b, decimal_places), zip(a_list, b_list)))


    def test_boxy_movement_active_update(self):
        global robot, ltf
        setup()

        # Get pose of base_footprint
        p, q = ltf.tf_transform("/map", "projection/base_footprint")
        self.assertAlmostEqualList([0, 0, 0], p)
        self.assertAlmostEqualList([0, 0, 0, 1], q)

        # Move robot
        robot.set_position_and_orientation([0.4, 0.7, 0], [0, 0, 0, 1])
        sleep(0.5)

        # Update the local transformer
        ltf.update_from_btr()

        # Get the updated pose of the robot
        p, q = ltf.tf_transform("/map", "projection/base_footprint")
        self.assertAlmostEqualList([0.4, 0.7, 0], p)
        self.assertAlmostEqualList([0, 0, 0, 1], q)

        clear()

    def test_boxy_movement_passive_update(self):
        global robot, ltf
        setup()

        # Get pose of base_footprint
        p, q = ltf.tf_transform("/map", "projection/base_footprint")
        self.assertAlmostEqualList([0, 0, 0], p)
        self.assertAlmostEqualList([0, 0, 0, 1], q)

        # Move robot
        robot.set_position_and_orientation([0.4, 0.7, 0], [0, 0, 0, 1])
        sleep(0.5)

        # Passively update the local transformer
        sleep(1)

        # Get the updated pose of the robot
        p, q = ltf.tf_transform("/map", "projection/base_footprint")
        self.assertAlmostEqualList([0.4, 0.7, 0], p)
        self.assertAlmostEqualList([0, 0, 0, 1], q)

        clear()

    def test_boxy_move_torso_active_update(self):
        global robot, ltf
        setup()

        p_before, _ = ltf.tf_transform("projection/base_link", "projection/triangle_base_link")
        robot.set_joint_state("triangle_base_joint", -0.2)
        sleep(0.5)
        ltf.update_from_btr()
        p_after, _ = ltf.tf_transform("projection/base_link", "projection/triangle_base_link")
        p_before_r = list(map(lambda n: round(n, 2), p_before))
        p_after_r = list(map(lambda n: round(n, 2), p_after))
        self.assertAlmostEqualList([p_before_r[0], p_before_r[1], p_before_r[2] - 0.2], p_after_r)

        robot.set_joint_state("triangle_base_joint", 0)
        clear()

    def test_boxy_move_torso_passive_update(self):
        global robot, ltf
        setup()

        p_before, _ = ltf.tf_transform("projection/base_link", "projection/triangle_base_link")
        robot.set_joint_state("triangle_base_joint", -0.2)
        sleep(0.5)
        sleep(0.5)
        p_after, _ = ltf.tf_transform("projection/base_link", "projection/triangle_base_link")
        p_before_r = list(map(lambda n: round(n, 2), p_before))
        p_after_r = list(map(lambda n: round(n, 2), p_after))
        self.assertAlmostEqualList([p_before_r[0], p_before_r[1], p_before_r[2] - 0.2], p_after_r)

        robot.set_joint_state("triangle_base_joint", 0)
        clear()

    def test_boxy_move_arm_active_update(self):
        global robot, ltf
        setup()

        # Get base link of left arm and tool frame
        link = robot_description.get_tool_frame("left")
        base_link = "calib_left_arm_base_link"

        # Save init pose of tool frame in base_link and global frame
        p_before, _ = ltf.tf_transform("projection/" + base_link, "projection/" + link)
        p_before_global, _ = ltf.tf_transform("/map", "projection/" + link)
        p_before_bullet, _ = robot.get_link_relative_to_other_link(base_link, link)
        p_before_bullet_global, _ = robot.get_link_position_and_orientation_tf(link)

        # Move left arm
        for joint_name, j_value in robot_description.get_static_joint_chain("left", "handover").items():
            robot.set_joint_state(joint_name, j_value)

        sleep(1)

        # Update TF
        ltf.update_from_btr()
        # Get updated pose of tool frame in base_link and global frame
        p_after, _ = ltf.tf_transform("projection/" + base_link, "projection/" + link)
        p_after_global, _ = ltf.tf_transform("/map", "projection/" + link)
        p_after_bullet, _ = robot.get_link_relative_to_other_link(base_link, link)
        p_after_bullet_global, _ = robot.get_link_position_and_orientation_tf(link)

        # Check if local pybullets and the tf-transformers poses are the same
        self.assertAlmostEqualList(p_before_bullet, p_before)
        self.assertAlmostEqualList(p_after_bullet, p_after)
        # Check if global pybullets and the tf-transformers poses are the same
        self.assertAlmostEqualList(p_before_bullet_global, p_before_global)
        self.assertAlmostEqualList(p_after_bullet_global, p_after_global)

        clear()

    def test_boxy_move_arm_passive_update(self):
        global robot, ltf
        setup()

        # Get base link of left arm and tool frame
        link = robot_description.get_tool_frame("left")
        base_link = "calib_left_arm_base_link"

        # Save init pose of tool frame in base_link and global frame
        p_before, _ = ltf.tf_transform("projection/" + base_link, "projection/" + link)
        p_before_global, _ = ltf.tf_transform("/map", "projection/" + link)
        p_before_bullet, _ = robot.get_link_relative_to_other_link(base_link, link)
        p_before_bullet_global, _ = robot.get_link_position_and_orientation_tf(link)

        # Move left arm
        for joint_name, j_value in robot_description.get_static_joint_chain("left", "handover").items():
            robot.set_joint_state(joint_name, j_value)
        sleep(1)

        # Update TF
        sleep(0.5)

        # Get updated pose of tool frame in base_link and global frame
        p_after, _ = ltf.tf_transform("projection/" + base_link, "projection/" + link)
        p_after_global, _ = ltf.tf_transform("/map", "projection/" + link)
        p_after_bullet, _ = robot.get_link_relative_to_other_link(base_link, link)
        p_after_bullet_global, _ = robot.get_link_position_and_orientation_tf(link)

        # Check if local pybullets and the tf-transformers poses are the same
        self.assertAlmostEqualList(p_before_bullet, p_before)
        self.assertAlmostEqualList(p_after_bullet, p_after)
        # Check if global pybullets and the tf-transformers poses are the same
        self.assertAlmostEqualList(p_before_bullet_global, p_before_global)
        self.assertAlmostEqualList(p_after_bullet_global, p_after_global)

        clear()


if __name__ == '__main__':
    world = BulletWorld()
    world.set_gravity([0, 0, -9.8])
    robot = Object("boxy", "robot", "../../resources/boxy.urdf")
    BulletWorld.robot = robot

    unittest.main()
