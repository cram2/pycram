import unittest

from unittest.mock import patch, MagicMock
from tf2_msgs.msg import TFMessage

from pycram.datastructures.enums import ExecutionType
from pycram.ros_utils.tf_broadcaster import TFBroadcaster


class TestTFBroadcaster(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Patching core dependencies
        cls.patcher_sleep = patch("pycram.ros_utils.tf_broadcaster.time.sleep", lambda x: None).start()
        cls.patcher_thread = patch("pycram.ros_utils.tf_broadcaster.threading.Thread.start", lambda x: None).start()
        cls.patcher_atexit = patch("pycram.ros_utils.tf_broadcaster.atexit.register", lambda x: None).start()

        cls.mock_world = patch("pycram.ros_utils.tf_broadcaster.World").start()
        cls.mock_time = patch("pycram.ros_utils.tf_broadcaster.Time").start()
        cls.mock_pose = patch("pycram.ros_utils.tf_broadcaster.PoseStamped").start()
        cls.mock_tfmsg = patch("pycram.ros_utils.tf_broadcaster.TFMessage", TFMessage).start()
        cls.mock_tf_publisher = MagicMock(name="tf_publisher")
        cls.mock_tf_static_publisher = MagicMock(name="tf_static_publisher")
        cls.mock_create_pub = patch("pycram.ros_utils.tf_broadcaster.create_publisher").start()
        cls.mock_create_pub.side_effect = [cls.mock_tf_static_publisher, cls.mock_tf_publisher]

        cls.mock_now = MagicMock()
        cls.mock_now.now.return_value = 42.0
        cls.mock_time.return_value = cls.mock_now


    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_initialization(self):
        broadcaster = TFBroadcaster()
        self.assertIsNotNone(broadcaster.tf_publisher)
        self.assertIsNotNone(broadcaster.tf_static_publisher)
        self.assertIs(broadcaster.tf_publisher, self.mock_tf_publisher)
        self.assertIs(broadcaster.tf_static_publisher, self.mock_tf_static_publisher)
        self.assertEqual(broadcaster.interval, 0.1)
        self.assertEqual(broadcaster.odom_frame, "odom")
        self.assertEqual(broadcaster.projection_namespace, ExecutionType.SIMULATED)

    def test_update_static_odom_calls_publish_pose(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.tf_publisher = MagicMock()
        broadcaster.tf_static_publisher = MagicMock()
        broadcaster.projection_namespace = ExecutionType.SIMULATED
        broadcaster._publish_pose = MagicMock()
        broadcaster.odom_frame = "odom"

        mock_pose = MagicMock()
        with patch("pycram.ros_utils.tf_broadcaster.PoseStamped.from_list", return_value=mock_pose):
            broadcaster._update_static_odom()
            broadcaster._publish_pose.assert_called_once_with("odom", mock_pose, static=True)

    def test_update_objects_publishes_all_objects_and_links(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.world = MagicMock()
        broadcaster.tf_publisher = MagicMock()
        broadcaster.tf_static_publisher = MagicMock()
        broadcaster.projection_namespace = ExecutionType.SIMULATED

        mock_obj = MagicMock()
        mock_obj.tf_frame = "obj1"
        mock_obj.link_name_to_id = {"link1": 0}
        mock_obj.get_link_tf_frame.return_value = "obj1/link1"
        broadcaster.world.objects = [mock_obj]

        broadcaster._publish_pose = MagicMock()
        broadcaster._update_objects()

        self.assertEqual(broadcaster._publish_pose.call_count, 2)
        self.assertEqual(broadcaster._publish_pose.call_args_list[0][0][0], "obj1")

    def test_update_objects_with_no_objects(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.world = MagicMock()
        broadcaster.world.objects = []
        broadcaster._publish_pose = MagicMock()

        broadcaster._update_objects()
        broadcaster._publish_pose.assert_not_called()

    def test_publish_pose_calls_correct_publisher(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.tf_publisher = MagicMock()
        broadcaster.tf_static_publisher = MagicMock()
        broadcaster.projection_namespace = ExecutionType.SIMULATED

        mock_pose = MagicMock()
        mock_pose.frame_id = "base"
        mock_transform = MagicMock()
        mock_transform.child_frame_id = "child"
        mock_transform.frame_id = "base"
        mock_pose.to_transform_stamped.return_value = mock_transform

        with patch("pycram.ros_utils.tf_broadcaster.TFMessage", return_value=TFMessage()):
            broadcaster._publish_pose("child", mock_pose, static=False)

        broadcaster.tf_publisher.publish.assert_called_once()

    def test_publish_loop_runs_once(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.kill_event = MagicMock()
        broadcaster.kill_event.is_set.side_effect = [False, True]
        broadcaster.update = MagicMock()
        broadcaster.interval = 0.01

        broadcaster._publish()

        broadcaster.update.assert_called_once()

    def test_stop_publishing(self):
        broadcaster = TFBroadcaster.__new__(TFBroadcaster)
        broadcaster.kill_event = MagicMock()
        broadcaster.thread = MagicMock()

        broadcaster._stop_publishing()
        broadcaster.kill_event.set.assert_called_once()
        broadcaster.thread.join.assert_called_once()


