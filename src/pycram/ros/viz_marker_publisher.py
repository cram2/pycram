import atexit
import threading
import time
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape
from ..datastructures.pose import Pose, Transform
from ..designator import ObjectDesignatorDescription
from ..datastructures.world import World


class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
        self.main_world = World.current_world if not World.current_world.is_prospection_world else World.current_world.world_sync.world

        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()

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
                geom = obj.get_link_geometry(link)
                if not geom:
                    continue
                msg = Marker()
                msg.header.frame_id = "map"
                msg.ns = obj.name
                msg.id = obj.link_name_to_id[link]
                msg.type = Marker.MESH_RESOURCE
                msg.action = Marker.ADD
                link_pose = obj.get_link_transform(link)
                if obj.get_link_origin(link) is not None:
                    link_origin = obj.get_link_origin_transform(link)
                else:
                    link_origin = Transform()
                link_pose_with_origin = link_pose * link_origin
                msg.pose = link_pose_with_origin.to_pose().pose

                color = [1, 1, 1, 1] if obj.link_name_to_id[link] == -1 else obj.get_link_color(link).get_rgba()

                msg.color = ColorRGBA(*color)
                msg.lifetime = rospy.Duration(1)

                if isinstance(geom, MeshVisualShape):
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + geom.file_name
                    msg.scale = Vector3(1, 1, 1)
                    msg.mesh_use_embedded_materials = True
                elif isinstance(geom, CylinderVisualShape):
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                elif isinstance(geom, BoxVisualShape):
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(*geom.size)
                elif isinstance(geom, SphereVisualShape):
                    msg.type = Marker.SPHERE
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.radius * 2)

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
        self.marker_array_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

        self.interval = interval
        self.log_message = None

    def publish(self, pose: Pose, color: Optional[List] = None, bw_object: Optional[ObjectDesignatorDescription] = None,
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
        rospy.loginfo(self.log_message)
        thread.join()

    def _publish(self, pose: Pose, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None,
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

            rospy.sleep(self.interval)

    def _publish_pose(self, name: str, pose: Pose, color: Optional[List] = None):
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

        color_rgba = ColorRGBA(*color)
        self._make_marker_array(name=name, marker_type=Marker.ARROW, marker_pose=pose,
                                marker_scales=(0.05, 0.05, 0.05), color_rgba=color_rgba)
        self.marker_array_pub.publish(self.marker_array)
        self.log_message = f"Pose '{name}' published"

    def _publish_object(self, name: Optional[str], pose: Pose, bw_object: ObjectDesignatorDescription):
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

    def _make_marker_array(self, name, marker_type: int, marker_pose: Pose, marker_scales: Tuple = (1.0, 1.0, 1.0),
                           color_rgba: ColorRGBA = ColorRGBA(*[1.0, 1.0, 1.0, 1.0]),
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
        new_marker.header.stamp = rospy.Time.now()
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

    def _update_marker(self, marker_id: int, new_pose: Pose) -> bool:
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
        rospy.logwarn(f"Marker {marker_id} not found for update")
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
            rospy.logerr('No name for object given, cannot remove marker')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)

        rospy.loginfo(f"Removed Marker '{name}'")

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker_array)

        rospy.loginfo('Removed all markers')
