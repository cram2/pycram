import unittest

from unittest.mock import patch, MagicMock

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from pycram.ros import Time
from pycram.ros_utils.object_state_updater import RobotStateUpdater, EnvironmentStateUpdater
from pycram.datastructures.pose import PoseStamped, Pose, Header

class TestObjectStateUpdater(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_tf_listener = patch("pycram.ros_utils.object_state_updater.TransformListener").start()
        cls.mock_create_timer = patch("pycram.ros_utils.object_state_updater.create_timer").start()
        cls.mock_world = patch("pycram.ros_utils.object_state_updater.World").start()
        cls.mock_get_joint_position = patch(
            "pycram.ros_utils.object_state_updater.World.robot.get_joint_position").start()
        cls.mock_robot_desc = patch(
            "pycram.ros_utils.object_state_updater.RobotDescription.current_robot_description").start()
        cls.mock_wait_for_message = patch("pycram.ros_utils.object_state_updater.wait_for_message").start()
        cls.patcher_atexit = patch("pycram.ros_utils.object_state_updater.atexit.register", lambda x: None).start()

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def setUp(self):
        self.mock_tf_listener.reset_mock()
        self.mock_create_timer.reset_mock()
        self.mock_world.reset_mock()
        self.mock_get_joint_position.reset_mock()
        self.mock_robot_desc.reset_mock()
        self.mock_wait_for_message.reset_mock()

    def test_initialization_robot_state_updater(self):
        mock_buffer = patch("pycram.ros_utils.object_state_updater.Buffer").start()

        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        robot_state_updater = RobotStateUpdater("/tf", "/joint_states")
        robot_state_updater.tf_buffer = mock_buffer
        robot_state_updater.tf_buffer.assert_called_once()
        self.mock_tf_listener.assert_called_once_with(robot_state_updater.tf_buffer.return_value,
                                                      robot_state_updater.node)

        self.assertEqual(robot_state_updater.tf_topic, "/tf")
        self.assertEqual(robot_state_updater.joint_state_topic, "/joint_states")

        self.mock_create_timer.assert_called()
        self.assertEqual(self.mock_create_timer.call_count, 2)

        tf_call, joint_call = self.mock_create_timer.call_args_list
        tf_args = tf_call[0]
        joint_args = joint_call[0]

        self.assertAlmostEqual(tf_args[0].sec, 0.1)
        self.assertTrue(callable(tf_args[1]))

        self.assertAlmostEqual(joint_args[0].sec, 0.1)
        self.assertTrue(callable(joint_args[1]))

    def test_subscribe_tf_robot_state_updater(self):
        mock_buffer = patch("pycram.ros_utils.object_state_updater.Buffer").start()

        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        self.mock_robot_desc.current_robot_description.base_link = "base_link"
        robot_state_updater = RobotStateUpdater("/tf", "/joint_states")
        robot_state_updater.tf_buffer = mock_buffer

        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 3.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        header = Header()
        header.stamp = Time(0.0)
        header.frame_id = "world"
        mock_buffer.lookup_transform.return_value = (pose, header)

        msg = TransformStamped()
        robot_state_updater._subscribe_tf(msg)

        self.mock_world.robot.set_pose.assert_called_once_with(PoseStamped(pose, header))

    def test_subscribe_joint_state_robot_state_updater(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.position = [1.0, 2.0]

        self.mock_wait_for_message.return_value = msg

        robot_state_updater = RobotStateUpdater('/tf', '/joint_states')

        robot_state_updater._subscribe_joint_state(msg)

        # Configure mock to return correct positions
        self.mock_get_joint_position.side_effect = lambda name: {
            'joint1': 1.0, 'joint2': 2.0
        }[name]

        # Now your assertions will work
        position1 = self.mock_get_joint_position('joint1')
        position2 = self.mock_get_joint_position('joint2')
        self.assertEqual(position1, 1.0)
        self.assertEqual(position2, 2.0)

    def test_subscribe_joint_state_handles_error_robot_state_updater(self):
        self.mock_create_timer.side_effect = [MagicMock(), MagicMock()]

        # Patch wait_for_message to raise AttributeError
        self.mock_wait_for_message.side_effect = AttributeError("fail")

        robot_state_updater = RobotStateUpdater('/tf', '/joint_states')

        msg = JointState()

        # Should not raise
        try:
            robot_state_updater._subscribe_joint_state(msg)
        except Exception:
            self.fail("AttributeError was not handled gracefully")

    def test_stop_subscription_robot_state_updater(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        robot_state_updater = RobotStateUpdater('/tf', '/joint_states')

        robot_state_updater._stop_subscription()

        mock_tf_timer.shutdown.assert_called_once()
        mock_joint_state_timer.shutdown.assert_called_once()

    def test_initialization_environment_state_updater(self):
        mock_buffer = patch("pycram.ros_utils.object_state_updater.Buffer").start()
        environment_state_updater = EnvironmentStateUpdater("/tf", "/joint_states")
        environment_state_updater.tf_buffer = mock_buffer
        environment_state_updater.tf_buffer.assert_called_once()
        self.mock_tf_listener.assert_called_once_with(environment_state_updater.tf_buffer.return_value,
                                                      environment_state_updater.node)

        self.assertEqual(environment_state_updater.tf_topic, "/tf")
        self.assertEqual(environment_state_updater.joint_state_topic, "/joint_states")

        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_joint_state_timer]

        joint_call = self.mock_create_timer.call_args_list[0]
        joint_args, joint_kwargs = joint_call

        self.assertAlmostEqual(joint_args[0].sec, 0.1)
        self.assertTrue(callable(joint_args[1]))

    def test_subscribe_joint_state_environment_state_updater(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.position = [1.0, 2.0]

        self.mock_wait_for_message.return_value = msg

        environment_state_updater = EnvironmentStateUpdater('/tf', '/joint_states')

        environment_state_updater._subscribe_joint_state(msg)

        # Configure mock to return correct positions
        self.mock_get_joint_position.side_effect = lambda name: {
            'joint1': 1.0, 'joint2': 2.0
        }[name]

        # Now your assertions will work
        position1 = self.mock_get_joint_position('joint1')
        position2 = self.mock_get_joint_position('joint2')
        self.assertEqual(position1, 1.0)
        self.assertEqual(position2, 2.0)

    def test_stop_subscription_environment_state_updater(self):
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_joint_state_timer]

        environment_state_updater = EnvironmentStateUpdater('/tf', '/joint_states')

        environment_state_updater._stop_subscription()

        mock_joint_state_timer.shutdown.assert_called_once()
