import unittest

from datetime import timedelta
from unittest.mock import patch, MagicMock
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from pycram.multirobot import RobotManager
from pycram.robot_description import RobotDescriptionManager
from pycram.ros_utils.robot_state_updater import WorldStateUpdater

class TestObjectStateUpdater(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_buffer = patch("pycram.ros_utils.robot_state_updater.Buffer").start()
        cls.mock_tf_listener = patch("pycram.ros_utils.robot_state_updater.TransformListener").start()
        cls.mock_world = patch("pycram.ros_utils.robot_state_updater.World").start()
        cls.mock_create_timer = patch("pycram.ros_utils.robot_state_updater.create_timer").start()
        cls.mock_robot_desc = patch(
            "pycram.ros_utils.robot_state_updater.RobotManager.robot_description").start()
        cls.mock_wait_for_message = patch("pycram.ros_utils.robot_state_updater.wait_for_message").start()
        cls.patcher_atexit = patch("pycram.ros_utils.robot_state_updater.atexit.register", lambda x: None).start()

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def setUp(self):
        self.mock_tf_listener.reset_mock()
        self.mock_create_timer.reset_mock()
        self.mock_world.reset_mock()
        self.mock_robot_desc.reset_mock()
        self.mock_wait_for_message.reset_mock()
        self.mock_buffer.reset_mock()

    def test_initialization(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        world_state_updater = WorldStateUpdater("/tf", "/joint_states")
        world_state_updater.tf_buffer = self.mock_buffer
        world_state_updater.tf_buffer.assert_called_once()
        self.mock_tf_listener.assert_called_once_with(world_state_updater.tf_buffer.return_value,
                                                      world_state_updater.node)

        self.assertEqual(world_state_updater.tf_topic, "/tf")
        self.assertEqual(world_state_updater.joint_state_topic, "/joint_states")

        self.mock_create_timer.assert_called()
        self.assertEqual(self.mock_create_timer.call_count, 2)

        update_rate = timedelta(milliseconds=100)
        tf_call, joint_call = self.mock_create_timer.call_args_list
        tf_args = tf_call[0]
        joint_args = joint_call[0]

        self.assertAlmostEqual(tf_args[0].sec, update_rate.total_seconds())
        self.assertTrue(callable(tf_args[1]))

        self.assertAlmostEqual(joint_args[0].sec, update_rate.total_seconds())
        self.assertTrue(callable(joint_args[1]))

        self.assertIsNone(world_state_updater.world)

    def test_subscribe_tf_ignores_prospection_world(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        self.mock_world.current_world.is_prospection_world = True

        world_state_updater = WorldStateUpdater("/tf", "/joint_states",
                                                world=self.mock_world)
        world_state_updater._subscribe_tf(MagicMock())

        self.assertFalse(self.mock_buffer.return_value.lookup_transform.called)

    def test_subscribe_tf_updates_robot_pose(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        mock_robot = MagicMock()
        mock_robot.name = "robot"
        mock_robot.tf_frame = "base_link"
        mock_robot.set_pose = MagicMock()
        mock_robot.is_an_environment = False
        mock_robot.is_an_object = False

        self.mock_robot_desc.current_robot_description.name = "robot"
        self.mock_robot_desc.current_robot_description.base_link = "base_link"

        mock_world = MagicMock()
        mock_world.is_prospection_world = False
        mock_world.objects = [mock_robot]

        RobotDescriptionManager().register_description(self.mock_robot_desc.current_robot_description)
        RobotManager.add_robot(mock_robot)

        with patch("pycram.ros_utils.robot_state_updater.World.current_world", mock_world):
            world_state_updater = WorldStateUpdater("/tf", "/joint_states", robot=mock_robot)
            world_state_updater.tf_buffer = self.mock_buffer

            # Simulated TF transform as position and orientation lists
            position = [1.0, 2.0, 3.0]
            orientation = [0.0, 0.0, 0.0, 1.0]
            self.mock_buffer.lookup_transform.return_value = (position, orientation)

            msg = TransformStamped()
            world_state_updater._subscribe_tf(msg)

            mock_robot.set_pose.assert_called_once()
            pose_arg = mock_robot.set_pose.call_args[0][0]
            self.assertEqual(pose_arg.pose.position.x, 1.0)
            self.assertEqual(pose_arg.pose.position.y, 2.0)
            self.assertEqual(pose_arg.pose.position.z, 3.0)
            self.assertEqual(pose_arg.pose.orientation.x, 0.0)
            self.assertEqual(pose_arg.pose.orientation.y, 0.0)
            self.assertEqual(pose_arg.pose.orientation.z, 0.0)
            self.assertEqual(pose_arg.pose.orientation.w, 1.0)


    def test_subscribe_joint_state(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        joint_msg = MagicMock(spec=JointState)
        joint_msg.name = ['joint_1', 'joint_2']
        joint_msg.position = [1.0, 2.0]

        self.mock_wait_for_message.return_value = joint_msg
        mock_robot = MagicMock()

        world_state_updater = WorldStateUpdater("/tf", "/joint_states",
                                                robot=mock_robot,
                                                world=self.mock_world)
        world_state_updater._subscribe_joint_state(joint_msg)

        mock_robot.set_multiple_joint_positions.assert_called_once_with({'joint_1': 1.0, 'joint_2': 2.0})

    def test_subscribe_joint_state_handles_attribute_error(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        self.mock_wait_for_message.side_effect = AttributeError

        world_state_updater = WorldStateUpdater("/tf", "/joint_states",
                                                world=self.mock_world)
        world_state_updater._subscribe_joint_state(MagicMock())

    def test_stop_subscription(self):
        mock_tf_timer = MagicMock()
        mock_joint_state_timer = MagicMock()
        self.mock_create_timer.side_effect = [mock_tf_timer, mock_joint_state_timer]

        world_state_updater = WorldStateUpdater("/tf", "/joint_states",
                                                world=self.mock_world)
        world_state_updater._stop_subscription()

        mock_tf_timer.shutdown.assert_called_once()
        mock_joint_state_timer.shutdown.assert_called_once()
