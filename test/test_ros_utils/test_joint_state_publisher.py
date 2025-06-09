import unittest

from unittest.mock import patch, MagicMock
from sensor_msgs.msg import JointState

from pycram.ros_utils.joint_state_publisher import JointStatePublisher


class DummyRobot:
    def __init__(self):
        self.joint_name_to_id = {"joint1": 0, "joint2": 1}
        self.joint_states = {'joint1': 1.0, 'joint2': 2.0}

    def get_joint_position(self, joint_name):
        return self.joint_states[joint_name]


class TestJointStatePublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Patch dependencies once for all tests
        cls.mock_world = patch("pycram.ros_utils.joint_state_publisher.World").start()
        cls.mock_time = patch("pycram.ros_utils.joint_state_publisher.Time").start()
        cls.mock_create_publisher = patch("pycram.ros_utils.joint_state_publisher.create_publisher").start()
        cls.patcher_sleep = patch("pycram.ros_utils.joint_state_publisher.time.sleep", lambda x: None).start()
        cls.patcher_thread =  patch("pycram.ros_utils.joint_state_publisher.threading.Thread.start",
                                         lambda x: None).start()
        cls.patcher_atexit = patch("pycram.ros_utils.joint_state_publisher.atexit.register", lambda x: None).start()

        cls.mock_world.robot = DummyRobot()

        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher
        cls.mock_time.return_value.now.return_value = 1.2

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_initialization(self):
        publisher = JointStatePublisher(joint_state_topic="/test_topic", interval=0.05)
        self.assertEqual(publisher.interval, 0.05)
        self.assertIs(publisher.joint_state_pub, self.mock_publisher)

    def test_publish_sends_joint_state(self):
        publisher = JointStatePublisher.__new__(JointStatePublisher)
        publisher.world = self.mock_world
        publisher.joint_state_pub = self.mock_publisher
        publisher.interval = 0.1
        publisher.kill_event = MagicMock()
        publisher.kill_event.is_set.side_effect = [False, True]

        publisher._publish()

        self.assertTrue(self.mock_publisher.publish.called)
        msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(msg, JointState)
        self.assertEqual(msg.name, ['joint1', 'joint2'])
        self.assertEqual(list(msg.position), [1.0, 2.0])
        self.assertEqual(msg.header.stamp, 1.2)

    def test_stop_publishing(self):
        publisher = JointStatePublisher.__new__(JointStatePublisher)
        publisher.kill_event = MagicMock()
        publisher.thread = MagicMock()

        publisher._stop_publishing()
        publisher.kill_event.set.assert_called_once()
        publisher.thread.join.assert_called_once()



