import unittest
import rclpy

from unittest.mock import patch, MagicMock
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from pycram.datastructures.dataclasses import MeshVisualShape, BoundingBoxCollection, AxisAlignedBoundingBox
from pycram.datastructures.pose import PoseStamped
from pycram.ros import Duration
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher, ManualMarkerPublisher, TrajectoryPublisher, \
    BoundingBoxPublisher, CoordinateAxisPublisher

class TestVizMarkerPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_create_publisher = patch("pycram.ros_utils.single_type_publisher.create_publisher").start()
        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher
        cls.mock_world = patch("pycram.ros_utils.viz_marker_publisher.World").start()
        cls.patcher_thread = patch("pycram.ros_utils.viz_marker_publisher.threading.Thread.start",
                                  lambda x: None).start()
        cls.patcher_atexit = patch("pycram.ros_utils.robot_state_updater.atexit.register", lambda x: None).start()

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def setUp(self):
        self.mock_create_publisher.reset_mock()
        self.mock_publisher.reset_mock()
        self.mock_world.reset_mock()

    def test_initialization(self):
        mock_world = self.mock_world
        mock_world.current_world.is_prospection_world = False
        viz = VizMarkerPublisher()
        self.assertEqual(viz.publisher, self.mock_publisher)
        self.assertEqual(viz.topic, "/pycram/viz_marker")
        self.assertEqual(viz.interval, 0.1)
        self.assertEqual(viz.reference_frame, "map")
        self.assertEqual(viz.use_prospection_world, False)
        self.assertEqual(viz.publish_visuals, False)
        self.assertIs(viz.main_world, mock_world.current_world)
        self.assertIs(viz.lock, viz.main_world.object_lock)

    def test_initialization_use_prospection_world_is_true(self):
        mock_world = self.mock_world
        viz = VizMarkerPublisher(use_prospection_world=True)
        self.assertEqual(viz.use_prospection_world, True)
        self.assertEqual(viz.topic, "/pycram/prospection_viz_marker")
        self.assertEqual(viz.main_world, mock_world.current_world.prospection_world)

    def test_initialization_is_prospection_world_is_true(self):
        mock_world = self.mock_world
        mock_world.current_world.is_prospection_world = True
        viz = VizMarkerPublisher()
        self.assertEqual(viz.main_world, mock_world.current_world.world_sync.world)

    def test_publish_marker_array(self):
        viz = VizMarkerPublisher()

        # Patch kill_event to stop loop after one iteration
        # patch the is_set method to return True immediately after the first call
        call_count = 0

        def is_set_side_effect():
            nonlocal call_count
            call_count += 1
            return call_count > 1  # stop after one iteration

        viz.kill_event.is_set = MagicMock(side_effect=is_set_side_effect)

        dummy_marker_array = MarkerArray()
        viz._make_marker_array = MagicMock(return_value=dummy_marker_array)

        viz.lock.acquire = MagicMock()
        viz.lock.release = MagicMock()

        viz.publisher.publish = MagicMock()

        # Patch time.sleep to avoid delay
        with patch("time.sleep", return_value=None):
            viz.visualize()  # call the method directly (runs one loop then stops)

        viz.lock.acquire.assert_called_once()
        viz.lock.release.assert_called_once()

        viz._make_marker_array.assert_called_once()

        viz.publisher.publish.assert_called_once_with(dummy_marker_array)

    def test_make_marker_array_mesh_geometry(self):
        self.mock_create_publisher.return_value = MagicMock()

        mock_geom = MagicMock(spec=MeshVisualShape)
        mock_geom.file_name = "/path/to/mesh.dae"
        mock_geom.scale = [1.0, 2.0, 3.0]
        mock_geom.__class__.__name__ = "MeshVisualShape"

        mock_obj = MagicMock()
        mock_obj.name = "test_object"
        mock_obj.link_name_to_id = {"link1": 1}
        mock_obj.get_link_geometry.return_value = mock_geom
        mock_obj.get_link_transform.return_value = MagicMock()
        mock_obj.get_link_origin.return_value = None
        mock_obj.get_link_origin_transform.return_value = MagicMock()
        mock_obj.get_link_color.return_value.get_rgba.return_value = [1.0, 0.0, 0.0, 1.0]

        mock_pose = MagicMock()
        mock_pose_stamped = MagicMock()
        mock_pose_stamped.pose.ros_message.return_value = "mock_pose"
        mock_pose.to_pose_stamped.return_value = mock_pose_stamped

        mock_link_origin = MagicMock()
        mock_pose.__mul__.return_value = mock_pose  # Needed fix
        mock_obj.get_link_transform.return_value = mock_pose
        mock_obj.get_link_origin_transform.return_value = mock_link_origin

        mock_world_instance = MagicMock()
        mock_world_instance.objects = [mock_obj]
        mock_world_instance.object_lock = MagicMock()
        self.mock_world.current_world = mock_world_instance
        mock_world_instance.is_prospection_world = False
        mock_world_instance.world_sync.world = mock_world_instance

        viz_pub = VizMarkerPublisher(publish_visuals=False)

        marker_array = viz_pub._make_marker_array(viz_pub._get_data())

        self.assertIsInstance(marker_array, MarkerArray)
        self.assertEqual(len(marker_array.markers), 1)

        marker = marker_array.markers[0]
        self.assertEqual(marker.ns, mock_obj.name)
        self.assertEqual(marker.id, mock_obj.link_name_to_id["link1"] * 10000)
        self.assertEqual(marker.type, Marker.MESH_RESOURCE)
        self.assertEqual(marker.mesh_resource, "file://" + mock_geom.file_name)
        self.assertEqual(marker.scale, Vector3(x=1.0, y=2.0, z=3.0))
        self.assertEqual(marker.color, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        self.assertEqual(marker.pose, mock_pose_stamped.pose.ros_message.return_value)

    def test_make_marker_array_skips_none_geometry(self):
        self.mock_create_publisher.return_value = MagicMock()

        mock_obj = MagicMock()
        mock_obj.name = "test_object"
        mock_obj.link_name_to_id = {"link1": 1}
        mock_obj.get_link_geometry.return_value = None
        mock_obj.get_link_transform.return_value = MagicMock()
        mock_obj.get_link_origin.return_value = None
        mock_obj.get_link_origin_transform.return_value = MagicMock()
        mock_obj.get_link_color.return_value.get_rgba.return_value = [1.0, 0.0, 0.0, 1.0]

        mock_world_instance = MagicMock()
        mock_world_instance.objects = [mock_obj]
        mock_world_instance.object_lock = MagicMock()
        self.mock_world.current_world = mock_world_instance
        mock_world_instance.is_prospection_world = False
        mock_world_instance.world_sync.world = mock_world_instance

        viz_pub = VizMarkerPublisher(publish_visuals=False)
        marker_array = viz_pub._make_marker_array(viz_pub._get_data())

        self.assertIsInstance(marker_array, MarkerArray)
        self.assertEqual(len(marker_array.markers), 0)

    def test_stop_publishing(self):
        viz = VizMarkerPublisher()
        viz.kill_event = MagicMock()
        viz.thread = MagicMock()

        viz._stop_publishing()
        viz.kill_event.set.assert_called_once()
        viz.thread.join.assert_called_once()

class TestManualMarkerPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_create_publisher = patch("pycram.ros_utils.single_type_publisher.create_publisher").start()
        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher

    @classmethod
    def tearDownClass(cls):
        patch.stopall()
        rclpy.shutdown()

    def setUp(self):
        self.publisher = ManualMarkerPublisher()
        self.pose1 = PoseStamped()
        self.pose1.header.frame_id = "base_link"
        self.pose1.pose.position.x = 1.0
        self.pose1.pose.position.y = 2.0
        self.pose1.pose.position.z = 3.0

        self.pose2 = PoseStamped()
        self.pose2.header.frame_id = "map"
        self.pose2.pose.position.x = 4.0
        self.pose2.pose.position.y = 5.0
        self.pose2.pose.position.z = 6.0

    def test_initialization(self):
        manual_marker_publisher = ManualMarkerPublisher()
        self.assertIsNone(manual_marker_publisher.start_time)
        self.assertEqual(manual_marker_publisher.publisher, self.mock_publisher)
        self.assertEqual(manual_marker_publisher.marker_array, MarkerArray())
        self.assertEqual(manual_marker_publisher.marker_overview, {})
        self.assertEqual(manual_marker_publisher.current_id, 0)
        self.assertEqual(manual_marker_publisher.interval, 0.1)

    def test_publish_pose_marker(self):
        with patch("pycram.ros_utils.viz_marker_publisher.loginfo") as mock_log:
            self.publisher.visualize(self.pose1)

        self.assertEqual(len(self.publisher.marker_array.markers), 1)
        marker = self.publisher.marker_array.markers[0]
        self.assertEqual(marker.ns, "pose_marker")
        self.assertEqual(marker.type, Marker.ARROW)
        self.assertEqual(marker.action, Marker.ADD)
        self.assertEqual(marker.pose, self.pose1.pose)
        self.assertEqual(marker.header.frame_id, self.pose1.header.frame_id)
        self.assertIn("pose_marker", self.publisher.marker_overview)

        self.assertTrue(self.mock_publisher.publish.called)
        mock_log.assert_called_with(f"Pose '{marker.ns}' published")

    def test_publish_object_marker(self):
        mock_bw_obj = MagicMock()
        mock_resolved = MagicMock()
        mock_resolved.name = "test_obj"
        mock_resolved.world_object.root_link.geometry.file_name = "mesh.stl"
        mock_bw_obj.resolve.return_value = mock_resolved

        with patch("pycram.ros_utils.viz_marker_publisher.loginfo") as mock_log:
            self.publisher.visualize(self.pose1, bw_object=mock_bw_obj)

        self.assertEqual(len(self.publisher.marker_array.markers), 1)
        marker = self.publisher.marker_array.markers[0]
        self.assertEqual(marker.ns, "test_obj")
        self.assertEqual(marker.type, Marker.MESH_RESOURCE)
        self.assertEqual(marker.mesh_resource, "file://mesh.stl")
        self.assertIn("test_obj", self.publisher.marker_overview)
        self.assertTrue(self.mock_publisher.publish.called)
        mock_log.assert_called_with("Object 'test_obj' published")

    def test_update_marker(self):
        self.publisher._publish_pose(name="test", pose=self.pose1)
        marker_id = self.publisher.marker_overview["test"]

        result = self.publisher._update_marker(marker_id, new_pose=self.pose2)

        self.assertTrue(result)
        marker = self.publisher.marker_array.markers[0]
        self.assertEqual(marker.pose, self.pose2)
        self.assertTrue(self.mock_publisher.publish.called)
        self.assertEqual(self.publisher.log_message, "Marker 'test' updated")

    def test_update_marker_not_found(self):
        with patch("pycram.ros_utils.viz_marker_publisher.logwarn") as mock_warn:
            result = self.publisher._update_marker(999, new_pose=PoseStamped())

        self.assertFalse(result)
        mock_warn.assert_called()

    def test_remove_marker_by_name(self):
        self.publisher._publish_pose(name="test", pose=self.pose1)

        with patch("pycram.ros_utils.viz_marker_publisher.loginfo") as mock_log:
            self.publisher.remove_marker(name="test")

        self.assertEqual(len(self.publisher.marker_array.markers), 0)
        self.assertNotIn("test", self.publisher.marker_overview)
        self.assertTrue(self.mock_publisher.publish.called)
        mock_log.assert_called_with("Removed Marker 'test'")

    def test_remove_marker_by_object(self):
        mock_bw_obj = MagicMock()
        mock_resolved = MagicMock()
        mock_resolved.name = "test_obj"
        mock_bw_obj.resolve.return_value = mock_resolved

        self.publisher._publish_object(name="test_obj", pose=self.pose1, bw_object=mock_bw_obj)

        with patch("pycram.ros_utils.viz_marker_publisher.loginfo") as mock_log:
            self.publisher.remove_marker(bw_object=mock_bw_obj)

        self.assertEqual(len(self.publisher.marker_array.markers), 0)
        self.assertNotIn("test_obj", self.publisher.marker_overview)
        mock_log.assert_called_with("Removed Marker 'test_obj'")

    def test_remove_marker_missing_name(self):
        with patch("pycram.ros_utils.viz_marker_publisher.logerr") as mock_err:
            self.publisher.remove_marker()

        mock_err.assert_called_with("No name for object given, cannot remove marker")

    def test_clear_all_marker(self):
        self.publisher._publish_pose(name="test", pose=self.pose1)

        with patch("pycram.ros_utils.viz_marker_publisher.loginfo") as mock_log:
            self.publisher.clear_all_marker()

        self.assertEqual(len(self.publisher.marker_array.markers), 1)
        for marker in self.publisher.marker_array.markers:
            self.assertEqual(marker.action, Marker.DELETE)
        self.assertEqual(self.publisher.marker_overview, {})
        self.assertTrue(self.mock_publisher.publish.called)
        mock_log.assert_called_with("Removed all markers")

class TestTrajectoryPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_create_publisher = patch("pycram.ros_utils.single_type_publisher.create_publisher").start()
        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_publisher(self):
        trajectory_pub = TrajectoryPublisher()
        self.assertEqual(trajectory_pub.publisher, self.mock_publisher)

    def test_visualize_trajectory(self):
        with patch("pycram.ros_utils.single_type_publisher.create_publisher") as mock_create_publisher:
            mock_publisher = MagicMock()
            mock_create_publisher.return_value = mock_publisher

            point_1 = PoseStamped()
            point_1.header.frame_id = "map"
            point_1.pose.position.x = 1.0
            point_1.pose.position.y = 2.0
            point_1.pose.position.z = 3.0

            point_2 = PoseStamped()
            point_2.header.frame_id = "world"
            point_2.pose.position.x = 4.0
            point_2.pose.position.y = 5.0
            point_2.pose.position.z = 6.0

            point_3 = PoseStamped()
            point_3.header.frame_id = "base"
            point_3.pose.position.x = 7.0
            point_3.pose.position.y = 8.0
            point_3.pose.position.z = 9.0

            trajectory = [point_1, point_2, point_3]

            trajectory_pub = TrajectoryPublisher()
            trajectory_pub.visualize(trajectory)

            mock_publisher.publish.assert_called_once()
            pub_msgs = mock_publisher.publish.call_args[0][0]
            assert isinstance(pub_msgs, MarkerArray)

            self.assertEqual(len(pub_msgs.markers), 2)
            self.assertEqual(pub_msgs.markers[0].header.frame_id, point_1.header.frame_id)
            self.assertEqual(pub_msgs.markers[1].header.frame_id, point_2.header.frame_id)
            self.assertEqual(pub_msgs.markers[0].points[0].x, point_1.pose.position.x)
            self.assertEqual(pub_msgs.markers[0].points[0].y, point_1.pose.position.y)
            self.assertEqual(pub_msgs.markers[0].points[0].z, point_1.pose.position.z)
            self.assertEqual(pub_msgs.markers[0].points[1].x, point_2.pose.position.x)
            self.assertEqual(pub_msgs.markers[0].points[1].y, point_2.pose.position.y)
            self.assertEqual(pub_msgs.markers[0].points[1].z, point_2.pose.position.z)
            self.assertEqual(pub_msgs.markers[1].points[0].x, point_2.pose.position.x)
            self.assertEqual(pub_msgs.markers[1].points[0].y, point_2.pose.position.y)
            self.assertEqual(pub_msgs.markers[1].points[0].z, point_2.pose.position.z)
            self.assertEqual(pub_msgs.markers[1].points[1].x, point_3.pose.position.x)
            self.assertEqual(pub_msgs.markers[1].points[1].y, point_3.pose.position.y)
            self.assertEqual(pub_msgs.markers[1].points[1].z, point_3.pose.position.z)

            for i, marker in enumerate(pub_msgs.markers):
                self.assertEqual(marker.ns, f"arrow_marker_{marker.id}")
                self.assertEqual(marker.action, Marker.ADD)
                self.assertEqual(marker.type, Marker.ARROW)
                self.assertEqual(marker.id, i)
                self.assertEqual(marker.scale.x, 0.01)
                self.assertEqual(marker.scale.y, 0.02)
                self.assertEqual(marker.scale.z, 0.02)
                self.assertEqual(marker.color, ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0))
                self.assertEqual(len(marker.points), 2)

class TestBoundingBoxPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_create_publisher = patch("pycram.ros_utils.single_type_publisher.create_publisher").start()
        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_publisher(self):
        bounding_box_pub = BoundingBoxPublisher()
        self.assertEqual(bounding_box_pub.publisher, self.mock_publisher)

    def test_visualization(self):
        with patch("pycram.ros_utils.single_type_publisher.create_publisher") as mock_create_publisher:
            mock_publisher = MagicMock()
            mock_create_publisher.return_value = mock_publisher

            box_1 = AxisAlignedBoundingBox(min_x=0.1, min_y=0.2, min_z=0.3, max_x=0.4, max_y=0.5, max_z=0.6)
            box_2 = AxisAlignedBoundingBox(min_x=0.3, min_y=0.4, min_z=0.5, max_x=0.7, max_y=0.8, max_z=0.9)

            boxes = BoundingBoxCollection([box_1, box_2])

            bbox_pub = BoundingBoxPublisher()
            bbox_pub.visualize(boxes)

            mock_publisher.publish.assert_called_once()
            pub_msgs = mock_publisher.publish.call_args[0][0]
            self.assertIsInstance(pub_msgs, MarkerArray)
            self.assertEqual(len(pub_msgs.markers), 2)

            for i in range(len(pub_msgs.markers)):
                # Check first marker
                marker = pub_msgs.markers[i]
                box = boxes.bounding_boxes[i]
                self.assertEqual(marker.header.frame_id, box.transform.frame_id)
                self.assertEqual(marker.id, i)
                self.assertEqual(marker.ns, f"cube_marker_{marker.id}")
                self.assertEqual(marker.action, Marker.ADD)
                self.assertEqual(marker.type, Marker.CUBE)
                self.assertEqual(marker.lifetime, Duration(60))
                self.assertEqual(marker.pose.position.x, box.transform.position.x)
                self.assertEqual(marker.pose.position.y, box.transform.position.y)
                self.assertEqual(marker.pose.position.z, box.transform.position.z)
                self.assertEqual(marker.scale.x, box.depth)
                self.assertEqual(marker.scale.y, box.width)
                self.assertEqual(marker.scale.z, box.height)
                self.assertEqual(marker.color, ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5))

class TestCoordinateAxisPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_create_publisher = patch("pycram.ros_utils.single_type_publisher.create_publisher").start()
        cls.mock_publisher = MagicMock()
        cls.mock_create_publisher.return_value = cls.mock_publisher

    @classmethod
    def tearDownClass(cls):
        patch.stopall()

    def test_publisher(self):
        coordinate_axis_pub = CoordinateAxisPublisher()
        self.assertEqual(coordinate_axis_pub.publisher, self.mock_publisher)

    def test_visualize(self):
        with patch("pycram.ros_utils.single_type_publisher.create_publisher") as mock_create_publisher:
            mock_publisher = MagicMock()
            mock_create_publisher.return_value = mock_publisher

            pose = PoseStamped()
            pose.frame_id = "map"
            pose.position.x = 1.0
            pose.position.y = 2.0
            pose.position.z = 3.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            coord_pub = CoordinateAxisPublisher()
            coord_pub.visualize([pose])

            mock_publisher.publish.assert_called_once()
            pub_msg = mock_publisher.publish.call_args[0][0]
            self.assertIsInstance(pub_msg, MarkerArray)
            self.assertEqual(len(pub_msg.markers), 3)  # One for each axis (X, Y, Z)

            axis_colors = [
                [1.0, 0.0, 0.0],  # X
                [0.0, 1.0, 0.0],  # Y
                [0.0, 0.0, 1.0]  # Z
            ]

            for i, marker in enumerate(pub_msg.markers):
                self.assertEqual(marker.header.frame_id, pose.frame_id)
                self.assertEqual(marker.type, Marker.LINE_LIST)
                self.assertEqual(marker.action, Marker.ADD)
                self.assertEqual(marker.lifetime, Duration(60))
                self.assertEqual(len(marker.points), 2)

                start = marker.points[0]
                end = marker.points[1]

                expected_ends = [
                    (pose.position.x + 0.1, pose.position.y, pose.position.z),  # X
                    (pose.position.x, pose.position.y + 0.1, pose.position.z),  # Y
                    (pose.position.x, pose.position.y, pose.position.z + 0.1)  # Z
                ]

                expected_end = expected_ends[i % 3]

                self.assertEqual(start.x, pose.position.x)
                self.assertEqual(start.y, pose.position.y)
                self.assertEqual(start.z, pose.position.z)
                self.assertEqual(end.x, expected_end[0])
                self.assertEqual(end.y, expected_end[1])
                self.assertEqual(end.z, expected_end[2])

                expected_color = axis_colors[i % 3]
                self.assertEqual(marker.color.r, expected_color[0])
                self.assertEqual(marker.color.g, expected_color[1])
                self.assertEqual(marker.color.b, expected_color[2])
                self.assertEqual(marker.color.a, 1.0)
