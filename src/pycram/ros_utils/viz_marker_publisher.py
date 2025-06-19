import atexit
import threading
import time
from functools import cached_property
from typing import List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Vector3, Point, Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape, \
    BoundingBox, BoundingBoxCollection, Color
from ..datastructures.enums import AxisIdentifier
from ..datastructures.pose import PoseStamped, TransformStamped, Quaternion
from ..datastructures.world import World
from ..designator import ObjectDesignatorDescription
from ..ros import  Duration, Time
from ..ros import  loginfo, logwarn, logerr
from ..ros import  create_publisher
from ..ros import  sleep
from ..tf_transformations import quaternion_multiply


class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1, reference_frame="map", use_prospection_world=False, publish_visuals=False):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        :param reference_frame: The reference frame of the visualization marker.
        :param use_prospection_world: If True, the visualization marker will be published for the prospection world.
        :param publish_visuals: If True, the visualization marker will be published.
        """
        self.use_prospection_world = use_prospection_world
        if self.use_prospection_world:
            self.topic_name = "/pycram/prospection_viz_marker"
        else:
            self.topic_name = topic_name
        self.interval = interval
        self.reference_frame = reference_frame

        self.pub = create_publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
        self.publish_visuals = publish_visuals
        if self.use_prospection_world:
            self.main_world = World.current_world.prospection_world
        else:
            self.main_world = World.current_world if not World.current_world.is_prospection_world else World.current_world.world_sync.world
        self.lock = self.main_world.object_lock
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            self.lock.acquire()
            marker_array = self._make_marker_array()
            self.lock.release()
            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        for obj in self.main_world.objects:
            if obj.name == "floor":
                continue
            for link in obj.link_name_to_id.keys():
                if self.publish_visuals:
                    geoms = obj.get_link_visual_geometry(link)
                else:
                    geoms = obj.get_link_geometry(link)
                if not isinstance(geoms, list):
                    geoms = [geoms]
                if len(geoms) < 1:
                    continue

                for i, geom in enumerate(geoms):
                    msg = Marker()
                    msg.header.frame_id = self.reference_frame
                    msg.ns = obj.name
                    msg.id = obj.link_name_to_id[link] * 10000 + i
                    msg.type = Marker.MESH_RESOURCE
                    msg.action = Marker.ADD
                    link_pose = obj.get_link_transform(link)
                    if obj.get_link_origin(link) is not None:
                        link_origin = obj.get_link_origin_transform(link)
                    else:
                        link_origin = TransformStamped.from_list()
                    link_pose_with_origin = link_pose * link_origin
                    msg.pose = link_pose_with_origin.to_pose_stamped().pose.ros_message()

                    color = obj.get_link_color(link).get_rgba()

                    msg.color = ColorRGBA(**dict(zip(["r", "g", "b","a"], color)))
                    if self.use_prospection_world:
                        msg.color.a = 0.5
                    msg.lifetime = Duration(5)

                    if isinstance(geom, MeshVisualShape):
                        msg.type = Marker.MESH_RESOURCE
                        msg.mesh_resource = "file://" + geom.file_name
                        if hasattr(geom, "scale") and geom.scale is not None:
                            msg.scale = Vector3(**dict(zip(["x", "y", "z"], geom.scale)))
                        else:
                            msg.scale = Vector3(x=1.0, y=1.0, z=1.0)
                        msg.mesh_use_embedded_materials = True
                    elif isinstance(geom, CylinderVisualShape):
                        msg.type = Marker.CYLINDER
                        msg.scale = Vector3(x=geom.radius * 2, y=geom.radius * 2, z=geom.length)
                    elif isinstance(geom, BoxVisualShape):
                        msg.type = Marker.CUBE
                        size = np.array(geom.size) * 2
                        msg.scale = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
                    elif isinstance(geom, SphereVisualShape):
                        msg.type = Marker.SPHERE
                        msg.scale = Vector3(x=geom.radius * 2, y=geom.radius * 2, z=geom.radius * 2)

                    marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()


class ManualMarkerPublisher:
    """
    Class to manually add and remove marker of objects and poses.
    """

    def __init__(self, topic_name: str = '/pycram/manual_marker', interval: float = 0.1):
        """
        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
        This Array is published with a rate of interval.

        :param topic_name: Name of the marker topic
        :param interval: Interval at which the marker should be published
        """
        self.start_time = None
        self.marker_array_pub = create_publisher(topic_name, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

        self.interval = interval
        self.log_message = None

    def publish(self, pose: PoseStamped, color: Optional[List] = None, bw_object: Optional[ObjectDesignatorDescription] = None,
                name: Optional[str] = None):
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param pose: Pose of the marker
        :param color: Color of the marker if no object is given
        :param bw_object: Object to add as a marker
        :param name: Name of the marker
        """

        if color is None:
            color = [1, 0, 1, 1]

        self.start_time = time.time()
        thread = threading.Thread(target=self._publish, args=(pose, bw_object, name, color))
        thread.start()
        loginfo(self.log_message)
        thread.join()

    def _publish(self, pose: PoseStamped, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None,
                 color: Optional[List] = None):
        """
        Publish the marker into the MarkerArray
        """
        stop_thread = False
        duration = 2

        while not stop_thread:
            if time.time() - self.start_time > duration:
                stop_thread = True
            if bw_object is None:
                self._publish_pose(name=name, pose=pose, color=color)
            else:
                self._publish_object(name=name, pose=pose, bw_object=bw_object)

            sleep(self.interval)

    def _publish_pose(self, name: str, pose: PoseStamped, color: Optional[List] = None):
        """
        Publish a Pose as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param color: Color of the marker
        """

        if name is None:
            name = 'pose_marker'

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        color_rgba = ColorRGBA(**dict(zip(["r", "g", "b","a"], color)))
        self._make_marker_array(name=name, marker_type=Marker.ARROW, marker_pose=pose,
                                marker_scales=(0.05, 0.05, 0.05), color_rgba=color_rgba)
        self.marker_array_pub.publish(self.marker_array)
        self.log_message = f"Pose '{name}' published"

    def _publish_object(self, name: Optional[str], pose: PoseStamped, bw_object: ObjectDesignatorDescription):
        """
        Publish an Object as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param bw_object: ObjectDesignatorDescription for the marker
        """

        bw_real = bw_object.resolve()

        if name is None:
            name = bw_real.name

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        path = bw_real.world_object.root_link.geometry.file_name

        self._make_marker_array(name=name, marker_type=Marker.MESH_RESOURCE, marker_pose=pose,
                                path_to_resource=path)

        self.marker_array_pub.publish(self.marker_array)
        self.log_message = f"Object '{name}' published"

    def _make_marker_array(self, name, marker_type: int, marker_pose: PoseStamped, marker_scales: Tuple = (1.0, 1.0, 1.0),
                           color_rgba: ColorRGBA = ColorRGBA(**dict(zip(["r", "g", "b","a"], [1.0, 1.0, 1.0, 1.0]))),
                           path_to_resource: Optional[str] = None):
        """
        Create a Marker and add it to the MarkerArray

        :param name: Name of the Marker
        :param marker_type: Type of the marker to create
        :param marker_pose: Pose of the marker
        :param marker_scales: individual scaling of the markers axes
        :param color_rgba: Color of the marker as RGBA
        :param path_to_resource: Path to the resource of a Bulletworld object
        """

        frame_id = marker_pose.header.frame_id
        new_marker = Marker()
        new_marker.id = self.current_id
        new_marker.header.frame_id = frame_id
        new_marker.ns = name
        # new_marker.header.stamp = Time.now()
        new_marker.type = marker_type
        new_marker.action = Marker.ADD
        new_marker.pose = marker_pose.pose
        new_marker.scale.x = marker_scales[0]
        new_marker.scale.y = marker_scales[1]
        new_marker.scale.z = marker_scales[2]
        new_marker.color.a = color_rgba.a
        new_marker.color.r = color_rgba.r
        new_marker.color.g = color_rgba.g
        new_marker.color.b = color_rgba.b

        if path_to_resource is not None:
            new_marker.mesh_resource = 'file://' + path_to_resource

        self.marker_array.markers.append(new_marker)
        self.marker_overview[name] = new_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id: int, new_pose: PoseStamped) -> bool:
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set

        :return: True if update was successful, False otherwise
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                self.log_message = f"Marker '{marker.ns}' updated"
                self.marker_array_pub.publish(self.marker_array)
                return True

        # Update was not successful
        logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None):
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if bw_object is not None:
            bw_real = bw_object.resolve()
            name = bw_real.name

        if name is None:
            logerr('No name for object given, cannot remove marker')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)

        loginfo(f"Removed Marker '{name}'")

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker_array)

        loginfo('Removed all markers')



class TrajectoryPublisher:
    """
    Publishes a trajectory as a MarkerArray to visualize it in rviz.
    """

    @cached_property
    def publisher(self):
        pub = create_publisher("/pycram/trajectory", MarkerArray)
        time.sleep(0.5) # this is needed to synchronize the publisher creation thread
        return pub

    def visualize_trajectory(self, trajectory: List[PoseStamped]):
        """
        Visualize a trajectory in rviz as a series of arrows.

        :param trajectory: The trajectory to visualize as a list of points where an arrow should be drawn
        from point to point.
        """
        marker_array = MarkerArray()
        for index, (p1, p2) in enumerate(zip(trajectory, trajectory[1:])):

            marker = Marker()
            marker.header.frame_id = p1.frame_id
            marker.id = index
            marker.ns = "trajectory_arrows"
            marker.action = Marker.ADD
            marker.type = Marker.ARROW
            marker.lifetime = Duration(60)

            marker_p1 = Point()
            marker_p1.x = p1.position.x
            marker_p1.y = p1.position.y
            marker_p1.z = p1.position.z

            marker_p2 = Point()
            marker_p2.x = p2.position.x
            marker_p2.y = p2.position.y
            marker_p2.z = p2.position.z

            marker.points.append(marker_p1)
            marker.points.append(marker_p2)

            marker.scale.x = 0.01
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)


class BoundingBoxPublisher:
    """
    Publishes a trajectory as a MarkerArray to visualize it in rviz.
    """

    id_counter = 0

    @cached_property
    def publisher(self):
        pub = create_publisher("/pycram/bounding_boxes", MarkerArray)
        time.sleep(0.5) # this is needed to synchronize the publisher creation thread
        return pub

    def visualize(self, boxes: BoundingBoxCollection, duration: Optional[float] = 60):
        """
        Visualize a collection of bounding boxes in rviz as a series of cubes.
        """
        marker_array = MarkerArray()
        for box in boxes:

            origin = box.transform.position

            marker = Marker()
            marker.header.frame_id = box.transform.frame_id
            marker.id = self.id_counter
            marker.ns = "bounding_boxes"
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            marker.lifetime = Duration(duration)

            marker.pose = Pose()
            marker.pose.position = origin.ros_message()

            marker.scale.x = box.depth
            marker.scale.y = box.width
            marker.scale.z = box.height

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5

            marker_array.markers.append(marker)
            self.id_counter += 1
        self.publisher.publish(marker_array)

class CoordinateAxisPublisher:
    """
    Publishes coordinate axes as a MarkerArray to visualize them in rviz.
    """

    id_counter = 10000

    @cached_property
    def publisher(self):
        pub = create_publisher("/pycram/coordinate_axis", MarkerArray)
        time.sleep(1) # this is needed to synchronize the publisher creation thread
        return pub

    def visualize(self, poses: List[PoseStamped], duration: Optional[float] = 60, length: float = 0.1):
        """
        Visualize coordinate axes in rviz to topic '/pycram/coordinate_axis'
        """
        marker_array = MarkerArray()
        for pose in poses:

            x_axis = self._create_line(pose, AxisIdentifier.X.value, duration, length,
                                       Color.from_rgb([1., 0., 0.]))
            y_axis = self._create_line(pose, AxisIdentifier.Y.value, duration, length,
                                       Color.from_rgb([0., 1., 0.]))
            z_axis = self._create_line(pose, AxisIdentifier.Z.value, duration, length,
                                       Color.from_rgb([0., 0., 1.]))
            marker_array.markers.append(x_axis)
            marker_array.markers.append(y_axis)
            marker_array.markers.append(z_axis)


        self.publisher.publish(marker_array)

    def _create_line(self, pose: PoseStamped, axis, duration, length, color):
        """
        Create a line marker to add to the marker array.

        :param pose: Starting pose of the line
        :param axis: Axis along which the line is set
        :param duration: Duration of the line marker
        :param length: Length of the line
        :param color: Optional color for the Line
        """

        def rotate_axis_by_quaternion(axis, quaternion):

            qx, qy, qz, qw = quaternion.to_list()

            # Represent axis as quaternion (x, y, z, 0)
            axis_quat = (*axis, 0)

            q = (qx, qy, qz, qw)
            q_inverse = (-qx, -qy, -qz, qw)

            # Rotate the vector
            rotated_quat = quaternion_multiply(quaternion_multiply(q, axis_quat), q_inverse)

            # The rotated vector is the vector part of the resulting quaternion
            return rotated_quat[:3]

        # Create a line marker for the axis
        line_marker = Marker()
        line_marker.header.frame_id = pose.frame_id
        line_marker.header.stamp = Time().now()
        line_marker.ns = f'axis_visualization_{self.id_counter}'
        line_marker.id = self.id_counter
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # Line width
        line_marker.color = ColorRGBA(**dict(zip(["r", "g", "b","a"], color.get_rgba())))
        line_marker.lifetime = Duration(duration)

        # Start point at the position specified by the pose (translation part)
        start_point = Point()
        start_point.x = pose.position.x
        start_point.y = pose.position.y
        start_point.z = pose.position.z

        quaternion = pose.orientation
        rotated_axis = rotate_axis_by_quaternion(axis, quaternion)

        # Calculate the end point by adding the rotated axis vector (scaled by length)
        end_point = Point()
        end_point.x = pose.position.x + (rotated_axis[0] * length)
        end_point.y = pose.position.y + (rotated_axis[1] * length)
        end_point.z = pose.position.z + (rotated_axis[2] * length)

        line_marker.points.append(start_point)
        line_marker.points.append(end_point)

        # Add the line marker to the MarkerArray
        self.id_counter += 1
        return line_marker

def plot_axis_in_rviz(poses: List[PoseStamped], duration: Optional[float] = 60, length: float = 0.3):
    def make_publisher():
        return CoordinateAxisPublisher()

    publisher = make_publisher()
    publisher.visualize(poses, duration, length)