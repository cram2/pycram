import atexit
import threading
import time

from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

from ..datastructures.world import World
from visualization_msgs.msg import MarkerArray, Marker
import rospy

from ..datastructures.pose import Transform
from ..datastructures.dataclasses import MeshVisualShape, CylinderVisualShape, BoxVisualShape, SphereVisualShape


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
