import atexit
import threading
import time

from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

from pycram.bullet_world import BulletWorld, Object
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import urdf_parser_py
from tf.transformations import quaternion_from_euler

from pycram.pose import Transform


class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the Bullet World
    """
    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the Bullet
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
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
        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name == "floor":
                continue
            for link in obj.links.keys():
                geom = obj.link_to_geometry[link]
                if not geom:
                    continue
                msg = Marker()
                msg.header.frame_id = "map"
                msg.ns = obj.name
                msg.id = obj.links[link]
                msg.type = Marker.MESH_RESOURCE
                msg.action = Marker.ADD
                link_pose = obj.get_link_pose(link).to_transform(link)
                if obj.urdf_object.link_map[link].collision.origin:
                    link_origin = Transform(obj.urdf_object.link_map[link].collision.origin.xyz,
                                            list(quaternion_from_euler(*obj.urdf_object.link_map[link].collision.origin.rpy)))
                else:
                    link_origin = Transform()
                link_pose_with_origin = link_pose * link_origin
                msg.pose = link_pose_with_origin.to_pose().pose

                color = [1, 1, 1, 1] if obj.links[link] == -1 else obj.get_color(link)

                msg.color = ColorRGBA(*color)
                msg.lifetime = rospy.Duration(1)

                if type(geom) == urdf_parser_py.urdf.Mesh:
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + geom.filename
                    msg.scale = Vector3(1, 1, 1)
                    msg.mesh_use_embedded_materials = True
                elif type(geom) == urdf_parser_py.urdf.Cylinder:
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                elif type(geom) == urdf_parser_py.urdf.Box:
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(*geom.size)
                elif type(geom) == urdf_parser_py.urdf.Sphere:
                    msg.type == Marker.SPHERE
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.radius * 2)

                marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()

