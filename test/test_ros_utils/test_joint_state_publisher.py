import unittest
from itertools import chain, repeat

from unittest.mock import patch, MagicMock

import rclpy.publisher
from sensor_msgs.msg import JointState
from pycram.ros_utils.joint_state_publisher import JointStatePublisher
from pycram.testing import BulletWorldTestCase, cleanup_ros


class DummyRobot:
    def __init__(self):
        self.joint_name_to_id = {"joint1": 0, "joint2": 1}
        self.joint_states = {'joint1': 1.0, 'joint2': 2.0}

    def get_joint_position(self, joint_name):
        return self.joint_states[joint_name]


class TestJointStatePublisher(BulletWorldTestCase):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.mock_publisher = MagicMock()
        cls.node = rclpy.create_node("test_node")

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_initialization(self):
        publisher = JointStatePublisher(self.world, self.node, joint_state_topic="/test_topic", interval=0.05)
        self.assertEqual(publisher.interval, 0.05)
        self.assertIsInstance(publisher.joint_state_pub, rclpy.publisher.Publisher)

        publisher._stop_publishing()

    def test_publish_sends_joint_state(self):
        publisher = JointStatePublisher(self.world, self.node)
        publisher.joint_state_pub = self.mock_publisher
        publisher.interval = 0.1
        publisher.kill_event = MagicMock()
        publisher.kill_event.is_set.side_effect = chain([False], repeat(True))

        publisher._publish()

        self.assertTrue(self.mock_publisher.publish.called)
        msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(msg, JointState)
        self.assertIn("torso_lift_joint", msg.name)
        self.assertIn("r_shoulder_pan_joint", msg.name)
        joint_to_position = dict(zip(msg.name, msg.position))
        self.assertAlmostEqual(joint_to_position["r_wrist_roll_joint"], self.world.state[self.world.get_degree_of_freedom_by_name("r_wrist_roll_joint")].position)
        self.assertAlmostEqual(joint_to_position["r_shoulder_pan_joint"], self.world.state[self.world.get_degree_of_freedom_by_name("r_shoulder_pan_joint")].position)

    def test_stop_publishing(self):
        publisher = JointStatePublisher(self.world, self.node)
        publisher.kill_event = MagicMock()
        publisher.thread = MagicMock()

        publisher._stop_publishing()
        publisher.kill_event.set.assert_called_once()
        publisher.thread.join.assert_called_once()



